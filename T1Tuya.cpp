/* ============================================================
 * T1Tuya - Tuya channel of the1Comm
 *
 * The cloud side of the mower. Two directions, both exit/enter here:
 *   cloud  -> device : the SDK calls __dev_obj_dp_cb() / __dev_raw_dp_cb(),
 *                      T1Tuya_OnObjDp() / T1Tuya_OnMoveDir() translate the
 *                      value into a "$C:xx" frame and hand it to the1Act.
 *   device -> cloud  : the event loop feeds every frame produced by the1 or
 *                      the1Act into T1Tuya::SendData(). The heart beat frame
 *                      carries the mower status in its "S:" field -> DP113.
 *
 * Everything of the channel lives in this single file: the file is copied into
 * the1Comm as is, so no extra .hpp/.cpp may appear next to it and
 * T1CommMain.cpp stays byte identical to the repository version.
 * The DP table (101~150) follows the Tuya requirement document, section 3.
 *
 * 2026.09.20: created by Lff（+AI）
 * 2026.09.21: modified by Lff（+AI）, absorb T1DpMap/T1DevOut into this file,
 *             add DP113 report and DP121 move direction
 * ============================================================ */

#include "T1Tuya.hpp"

/* Cloud -> device: the1Act consumes /theone_c2a. The queue name, the message
 * layout and the command type all come from the1Comm itself, so this include
 * keeps the file portable to that repository. */
#include "T1IPCComm2Act.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>

/* The Tuya SDK is plain C, its headers need C linkage */
extern "C" {
#include "tuya_cloud_types.h"
#include "tuya_cloud_com_defs.h"
#include "tuya_cloud_wifi_defs.h"
#include "tuya_iot_com_api.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_wifi_netcfg.h"
#include "tuya_wifi_reset.h"
#include "tuya_wifi_link.h"
#include "tuya_bt.h"
#include "tal_bluetooth.h"
#include "tal_system.h"
#include "tal_log.h"
#include "tuya_error_code.h"
#include "tuya_svc_upgrade.h"
#include "ws_db_gw.h"
}

/* ------------------------------------------------------------
 * Authorization info, from the Tuya developer platform
 * ("Product -> Hardware" page of this PID)
 * ------------------------------------------------------------ */
#ifndef T1_TUYA_PRODUCT_KEY
#define T1_TUYA_PRODUCT_KEY     "lcuqw3h96rgtkhrz"
#endif

#ifndef T1_TUYA_FIRMWARE_KEY
#define T1_TUYA_FIRMWARE_KEY    T1_TUYA_PRODUCT_KEY
#endif

#ifndef T1_TUYA_SW_VER
#define T1_TUYA_SW_VER          "1.0.0"
#endif

#ifndef T1_TUYA_UUID
#define T1_TUYA_UUID            "uuidfbc9b28086ee2c9a"
#endif

#ifndef T1_TUYA_AUTH_KEY
#define T1_TUYA_AUTH_KEY        "dCKOXmMhRbh8XeXGWKz0ZnFFjBmlHBYI"
#endif

/* ------------------------------------------------------------
 * Storage / log paths.
 * The SDK keeps its KV database and log files here, the paths must be on a
 * writable partition.
 * ------------------------------------------------------------ */
#define T1_TUYA_STORAGE_PATH    "/app/TheOne/tuya_x3m/"
#define T1_TUYA_LOG_PATH        "/app/TheOne/tuya_x3m/log/"

/* OTA package staging: downloaded as *.tmp, renamed on success only */
#define T1_TUYA_OTA_DIR         "/app/TheOne/tuya_x3m/"
#define T1_TUYA_OTA_TMP         T1_TUYA_OTA_DIR "ota_package.zip.tmp"
#define T1_TUYA_OTA_FINAL       T1_TUYA_OTA_DIR "ota_package.zip"

/* Used when the primary storage path is not writable (dev board without the
 * /app partition, or a root owned directory) */
#define T1_TUYA_FALLBACK_BASE   "/tmp/tuya_x3m/"

/* ------------------------------------------------------------
 * Store of the SDK: it keeps its device identity, wifi state and binding in a
 * key/value store of three files.
 *     tuya_user.db        main store
 *     tuya_user.db_bak    last good copy of it, written by the SDK itself
 *     tuya_enckey.db      key the store is encrypted with
 * Header of a store, measured on the board:
 *     offset  0.. 3   checksum, it changes with every write - NOT a magic number,
 *                     do not test it (6c 23 61 ae on an empty store, b7 57 1d a3
 *                     on a populated one; testing it rejects every healthy store)
 *     offset  4.. 7   format version (2)
 *     offset  8..11   number of records (8 on an empty store, 21 on a full one)
 *     offset 12..15   size of the data area (0x1C70 = 28700 bytes)
 *     offset 16..19   marker, always AA 55 AA 55 - the only field worth testing
 * The numbers below were measured on the board: key 16 bytes, a healthy store
 * 31236 bytes (35220 once the device is provisioned and bound).
 * ------------------------------------------------------------ */
#define T1_TUYA_KV_FILE         "tuya_user.db"
#define T1_TUYA_KV_BAK_FILE     "tuya_user.db_bak"
#define T1_TUYA_KV_KEY_FILE     "tuya_enckey.db"
#define T1_TUYA_KV_KEY_BYTES    16              /* the key file is exactly this long */
#define T1_TUYA_KV_MIN_BYTES    1024            /* below this a store is clearly truncated */
#define T1_TUYA_KV_MARK_OFFS    16              /* offset of the marker */
#define T1_TUYA_KV_MARK         "\xAA\x55\xAA\x55"  /* compared as bytes, no endian trap */

static char s_storagePath[128] = T1_TUYA_STORAGE_PATH;
static char s_logPath[128]     = T1_TUYA_LOG_PATH;

/* make sure a directory exists and can really be written by this process
 * dir  = directory path
 * mode = permission bits for the mkdir() call, e.g. 0755
 * return:  0, directory ready;   -1, not a directory / not writable / stat fail
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static int T1Tuya_EnsureDirWritable(const char *dir, mode_t mode)
{
    int debug=1;
    struct stat st;

    if ( dir==NULL || dir[0]=='\0' ) return -1;

    /* EEXIST is fine, the directory may already be there */
    if ( mkdir(dir, mode)!=0 && errno!=EEXIST ) {
        if ( debug>0 ) printf("T1Tuya-EnsureDir: mkdir(%s) failed: %s\n", dir, strerror(errno));
        return -1;
    }

    if ( stat(dir, &st)!=0 ) {
        if ( debug>0 ) printf("T1Tuya-EnsureDir: stat(%s) failed: %s\n", dir, strerror(errno));
        return -1;
    }
    if ( !S_ISDIR(st.st_mode) ) {
        if ( debug>0 ) printf("T1Tuya-EnsureDir: %s exists but is NOT a directory\n", dir);
        return -1;
    }

    /* mkdir() can succeed while the directory is still not writable by us
     * (mode is masked by umask, or the directory is owned by root) */
    if ( access(dir, W_OK)!=0 ) {
        if ( debug>0 ) {
            printf("T1Tuya-EnsureDir: %s NOT writable: %s\n", dir, strerror(errno));
            printf("T1Tuya-EnsureDir:   dir owner uid=%d mode=%04o, current uid=%d gid=%d\n",
                   (int)st.st_uid, (int)(st.st_mode & 07777), (int)getuid(), (int)getgid());
            printf("T1Tuya-EnsureDir:   fix with: sudo chown -R %d:%d %s\n",
                   (int)getuid(), (int)getgid(), dir);
        }
        return -1;
    }
    return 0;
}

/* tell whether one store file looks like a store: present, not obviously truncated
 * and carrying the marker at the offset it belongs to. Only the marker is tested;
 * the first 4 bytes are a checksum that changes with every write
 * path = full path of the file
 * return:  true, the file looks like an intact store
 * 2026.09.21: created by Lff（+AI）
 * 2026.09.21: modified by Lff（+AI），magic test dropped (it is a checksum) and the
 *             marker is compared as bytes to avoid the little endian trap
 */
static bool T1Tuya_KvLooksOk(const char *path)
{
    struct stat   st;
    unsigned char head[20];
    FILE         *fp=NULL;
    bool          ok=false;

    if ( path==NULL ) return false;
    if ( stat(path, &st)!=0 ) return false;
    if ( st.st_size<T1_TUYA_KV_MIN_BYTES ) return false;

    fp=fopen(path, "rb");
    if ( fp==NULL ) return false;
    if ( fread(head, 1, sizeof(head), fp)==sizeof(head) ) {
        if ( memcmp(head + T1_TUYA_KV_MARK_OFFS, T1_TUYA_KV_MARK, 4)==0 ) ok=true;
    }
    fclose(fp);
    return ok;
}

/* decide what to do with the store on disk and act on it right away
 * Has to run BEFORE the SDK opens the store (tuya_iot_init_params), that is why
 * it sits in the middle of Init().
 * The store is only ever touched when it is OBVIOUSLY not a store. Judging a
 * usable store as broken costs the binding, the grid map and the logs and sends
 * somebody out to re-provision; judging a broken one as usable costs nothing but
 * one more netcfg run. On top of that the SDK builds a fresh store and resets its
 * gateway state by itself when it cannot read one (measured: it logs "enter reset
 * factory"), so wiping is not a favour it needs from us.
 * The messages below use printf on purpose: this function runs BEFORE
 * tuya_iot_init_params(), where the SDK log backend is not up yet and TAL_PR_*
 * output is silently dropped - the same reason the other pre-init messages in
 * Init() use printf as well.
 * return:  1, store usable (possibly restored from the backup);
 *           0, the files were not a store at all and were removed, the SDK builds
 *              a fresh one and the device has to be provisioned again;
 *          -1, nothing to do (fresh device), or not recoverable but left alone
 * 2026.09.21: created by Lff（+AI）
 * 2026.09.21: modified by Lff（+AI）, only files that obviously are not a store are
 *             touched, a doubtful store is always left alone
 */
static int T1Tuya_KvStoreCheck(void)
{
    char        db_path[192];
    char        bak_path[192];
    char        key_path[192];
    struct stat st;
    long long   key_size=0;
    long long   db_size=0;
    bool        key_here=false;
    bool        key_ok=false;
    bool        db_here=false;
    bool        bak_here=false;
    bool        db_ok=false;
    bool        bak_ok=false;

    snprintf(db_path,  sizeof(db_path),  "%s%s", s_storagePath, T1_TUYA_KV_FILE);
    snprintf(bak_path, sizeof(bak_path), "%s%s", s_storagePath, T1_TUYA_KV_BAK_FILE);
    snprintf(key_path, sizeof(key_path), "%s%s", s_storagePath, T1_TUYA_KV_KEY_FILE);

    key_here=(stat(key_path, &st)==0);
    key_size=key_here ? (long long)st.st_size : 0;
    key_ok  =(key_here && key_size==T1_TUYA_KV_KEY_BYTES);
    db_here =(access(db_path,  F_OK)==0);
    bak_here=(access(bak_path, F_OK)==0);
    db_ok   =T1Tuya_KvLooksOk(db_path);
    bak_ok  =T1Tuya_KvLooksOk(bak_path);
    db_size =0;
    if ( db_here && stat(db_path, &st)==0 ) db_size=(long long)st.st_size;

    /* no store file at all: a fresh device, the SDK creates everything. A left
     * over key file is harmless and is left where it is. */
    if ( !db_here && !bak_here ) {
        printf("T1Tuya-KvCheck: no store yet, a fresh one will be created\n");
        return -1;
    }

    /* the normal case: the store is there and looks like a store - hands off */
    if ( db_ok ) {
        printf("T1Tuya-KvCheck: store ok, reuse it\n");
        return 1;
    }

    printf("T1Tuya-KvCheck: main store does not look like a store (size=%lld, key %s)\n",
           db_size, key_ok ? "ok" : "not ok");

    /* second chance: the backup the SDK keeps next to the store. It only helps
     * when the key itself is fine. */
    if ( key_ok && bak_ok ) {
        unlink(db_path);
        if ( rename(bak_path, db_path)==0 ) {
            printf("T1Tuya-KvCheck: restored the store from the backup, reuse it\n");
            return 1;
        }
        printf("T1Tuya-KvCheck: restore from the backup failed: %s, left as is\n",
               strerror(errno));
        return -1;
    }

    /* Only a file that can not hold a store at all is removed: a healthy store
     * measures tens of kilobytes, so anything below the minimum is truncated
     * rubbish and no data can be lost by removing it. A large file that merely
     * lacks our marker could still be a store of an unknown layout, so it is left
     * alone - losing the binding, the grid map and the logs, and sending somebody
     * out to re-provision, costs far more than one more netcfg run. */
    if ( db_here && db_size<T1_TUYA_KV_MIN_BYTES ) {
        printf("T1Tuya-KvCheck: store is truncated rubbish, wiped -> provisioning needed again\n");
        unlink(db_path);
        unlink(bak_path);
        unlink(key_path);
        return 0;
    }

    printf("T1Tuya-KvCheck: store not recoverable but left untouched on purpose\n");
    return -1;
}

/* ------------------------------------------------------------
 * Device -> cloud: DP report helpers.
 * Thin wrappers over the SDK report calls so that the rest of the file only
 * needs "dpid + value". json_async only queues the report, the return value
 * does not mean the cloud already received it.
 * ------------------------------------------------------------ */

/* report one bool DP
 * dpid  = function point id, must match the platform definition
 * value = 0 (false) / 1 (true)
 * return: OPRT_OK on success, otherwise a tuya_error_code.h value
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
int T1Tuya_ReportDpBool(unsigned char dpid, int value)
{
    TY_OBJ_DP_S dp;
    memset(&dp, 0, sizeof(dp));
    dp.dpid=dpid;
    dp.type=PROP_BOOL;
    dp.value.dp_bool=(value!=0) ? TRUE : FALSE;
    dp.time_stamp=0;
    return dev_report_dp_json_async(NULL, &dp, 1);
}

/* report one value DP
 * dpid  = function point id
 * value = integer value, range is defined by the platform
 * return: OPRT_OK on success, otherwise a tuya_error_code.h value
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
int T1Tuya_ReportDpValue(unsigned char dpid, int value)
{
    TY_OBJ_DP_S dp;
    memset(&dp, 0, sizeof(dp));
    dp.dpid=dpid;
    dp.type=PROP_VALUE;
    dp.value.dp_value=value;
    dp.time_stamp=0;
    return dev_report_dp_json_async(NULL, &dp, 1);
}

/* report one raw DP (log blocks, grid map, ...)
 * dpid = function point id, len = byte count of data
 * return: OPRT_OK on success, otherwise a tuya_error_code.h value
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
int T1Tuya_ReportDpRaw(unsigned char dpid, const unsigned char *data, unsigned len)
{
    if ( data==NULL || len==0 ) return OPRT_INVALID_PARM;
    return dev_report_dp_raw_sync_extend(NULL, dpid, data, len, 3, TRUE);
}

/* ============================================================
 * Device side endpoint - cloud -> device
 *
 * Was T1DevOut.hpp/.cpp, moved in on 2026.09.21 so that the file set of this
 * app equals the one of the1Comm: T1CommMain.cpp stays untouched and no extra
 * file has to be carried back on merge.
 *
 * Only the WRITE end of /theone_c2a is opened here. /theone_a2c is never
 * touched on purpose: mqueue is a competing-consumer queue, a second reader
 * would steal the status frames that the T1IPCComm2Act owned by T1CommMain
 * is supposed to receive.
 * ============================================================ */

static mqd_t s_actOut=(mqd_t)-1;

/* open the write end of the comm2act queue, lazily and only once
 * return:  0, ready;   -1, queue does not exist yet (the1Act not started)
 * 2026.09.21: moved from T1DevOut.cpp by Lff（+AI）
 */
static int T1Tuya_EnsureActOut(void)
{
    int debug=1;

    if ( s_actOut!=(mqd_t)-1 ) return 0;

    s_actOut=mq_open(COMM2ACT_QUEUE, O_WRONLY | O_NONBLOCK);
    if ( s_actOut==(mqd_t)-1 ) {
        if ( debug>0 ) printf("T1Tuya-EnsureActOut: open %s failed: %s\n",
                              COMM2ACT_QUEUE, strerror(errno));
        return -1;
    }
    if ( debug>0 ) printf("T1Tuya-EnsureActOut: %s write end ready\n", COMM2ACT_QUEUE);
    return 0;
}

/* push one frame to the1Act, which forwards it to the mower board
 * frame = NUL terminated text frame, truncated to ACTCOMM_MSG_MAX_SIZE-1 bytes
 * return:  0, queued;   -1, invalid frame / queue not ready / queue full
 * 2026.09.21: moved from T1DevOut.cpp by Lff（+AI）
 */
static int T1Tuya_SendToDevice(const char *frame)
{
    int debug=1;
    ActCommMessage msg;
    int len=0;

    if ( frame==NULL ) return -1;

    if ( T1Tuya_EnsureActOut()!=0 ) {
        if ( debug>0 ) printf("T1Tuya-SendToDevice: act queue not ready, drop: %s\n", frame);
        return -1;
    }

    memset(&msg, 0, sizeof(msg));
    msg.type=ACTCOMM_CMD_APP;
    len=(int)strlen(frame);
    if ( len>ACTCOMM_MSG_MAX_SIZE-1 ) len=ACTCOMM_MSG_MAX_SIZE-1;
    memcpy(msg.data, frame, len);

    if ( mq_send(s_actOut, (const char *)&msg, sizeof(msg), 0)!=0 ) {
        if ( debug>0 ) printf("T1Tuya-SendToDevice: mq_send failed: %s\n", strerror(errno));
        return -1;
    }
    if ( debug>0 ) printf("T1Tuya-SendToDevice: [cloud->device] %s\n", frame);
    return 0;
}

/* ============================================================
 * DP table and dispatch
 *
 * Was T1DpMap.hpp/.cpp, moved in on 2026.09.21.
 * The table is kept 1:1 with "Product -> Function definition" of the Tuya
 * developer platform: a wrong id, type or direction lets the SDK drop the DP
 * silently. The per-dpid action mapping follows the requirement document,
 * every case still open is marked TODO - guessing there would move the mower.
 * ============================================================ */

/* DP ids, requirement document 101~150 (115~149 reserved) */
#define DP_ID_BLADE_LIFE        101     /* blade life, value 0-2000 h, write 0 resets */
#define DP_ID_BATTERY_CYCLES    102     /* battery charge cycles, value 0-2000, write 0 resets */
#define DP_ID_RAIN_MOW          103     /* rain mowing switch, bool */
#define DP_ID_RAIN_RESUME       104     /* resume after rain, bool */
#define DP_ID_RAIN_DELAY        105     /* rain delay, value 5-150 min, step 5 */
#define DP_ID_EDGE_DISTANCE     106     /* edge mowing distance, value -200~200 cm */
#define DP_ID_RANDOM_MOW        107     /* random mowing switch, bool */
#define DP_ID_BLE_SWITCH        108     /* BLE switch for near field control, bool */
#define DP_ID_HEDGEHOG_GUARD    109     /* hedgehog protection, bool */
#define DP_ID_RANDOM_MODE       110     /* random mowing mode, bool */
#define DP_ID_ERROR_LOG         111     /* error log, raw 20 x 7 bytes, report only */
#define DP_ID_WORK_LOG          112     /* work log, raw 10 entries, report only */
#define DP_ID_DEVICE_STATUS     113     /* device status, value 0 idle/1 out/2 back/3 mowing/4 pause, report only */
#define DP_ID_LAWN_PROGRESS     114     /* lawn coverage progress, raw grid map, report only */
#define DP_ID_MOVE_DIR          121     /* move direction, value 0 stop/1 fwd/2 back/3 left/4 right, rw */
#define DP_ID_ERROR_REPORT_V2   150     /* error report v2, raw 10 x 8 bytes, pushed in real time */

/* Same values as the SDK PROP_xxx (tuya_cloud_com_defs.h:258-262) */
#define DP_KIND_BOOL    0
#define DP_KIND_VALUE   1
#define DP_KIND_RAW     2

/* one row of the DP table */
typedef struct {
    unsigned char dpid;     /* function point id */
    int           kind;     /* DP_KIND_BOOL / DP_KIND_VALUE / DP_KIND_RAW */
    int           rw;       /* 1 = writable from the cloud;  0 = report only */
    const char   *name;     /* platform name, used for log and table comparison */
} T1DpDesc;

static const T1DpDesc s_dpTable[] = {
    { DP_ID_BLADE_LIFE,      DP_KIND_VALUE, 1, "blade life"          },
    { DP_ID_BATTERY_CYCLES,  DP_KIND_VALUE, 1, "battery cycles"      },
    { DP_ID_RAIN_MOW,        DP_KIND_BOOL,  1, "rain mowing"         },
    { DP_ID_RAIN_RESUME,     DP_KIND_BOOL,  1, "rain resume"         },
    { DP_ID_RAIN_DELAY,      DP_KIND_VALUE, 1, "rain delay"          },
    { DP_ID_EDGE_DISTANCE,   DP_KIND_VALUE, 1, "edge distance"       },
    { DP_ID_RANDOM_MOW,      DP_KIND_BOOL,  1, "random mowing"       },
    { DP_ID_BLE_SWITCH,      DP_KIND_BOOL,  1, "ble switch"          },
    { DP_ID_HEDGEHOG_GUARD,  DP_KIND_BOOL,  1, "hedgehog guard"      },
    { DP_ID_RANDOM_MODE,     DP_KIND_BOOL,  1, "random mode"         },
    { DP_ID_ERROR_LOG,       DP_KIND_RAW,   0, "error log"           },
    { DP_ID_WORK_LOG,        DP_KIND_RAW,   0, "work log"            },
    { DP_ID_DEVICE_STATUS,   DP_KIND_VALUE, 0, "device status"       },
    { DP_ID_LAWN_PROGRESS,   DP_KIND_RAW,   0, "lawn progress"       },
    { DP_ID_MOVE_DIR,        DP_KIND_VALUE, 1, "move direction"      },
    { DP_ID_ERROR_REPORT_V2, DP_KIND_RAW,   0, "error report v2"     },
};

static const int s_dpTableCount=(int)(sizeof(s_dpTable) / sizeof(s_dpTable[0]));

/* look up one DP description
 * dpid = function point id
 * return:  pointer into the static table, NULL when the id is not known
 * 2026.09.21: moved from T1DpMap.cpp by Lff（+AI）
 */
static const T1DpDesc *T1Tuya_DpFind(unsigned char dpid)
{
    int i=0;
    for ( i=0; i<s_dpTableCount; i++ ) {
        if ( s_dpTable[i].dpid==dpid ) return &s_dpTable[i];
    }
    return NULL;
}

/* print the whole DP table once at startup, to compare it with the platform
 * 2026.09.21: moved from T1DpMap.cpp by Lff（+AI）
 */
static void T1Tuya_DumpDpTable(void)
{
    int debug=1;
    int i=0;

    if ( debug>0 ) {
        printf("T1Tuya-DumpDpTable: %d dps registered\n", s_dpTableCount);
    }
    for ( i=0; i<s_dpTableCount; i++ ) {
        if ( debug>0 ) {
            printf("T1Tuya-DumpDpTable: dpid=%-3u kind=%d rw=%d name=%s\n",
                   s_dpTable[i].dpid, s_dpTable[i].kind, s_dpTable[i].rw, s_dpTable[i].name);
        }
    }
}

/* cloud -> device: handle one bool/value/enum DP
 * dpid  = function point id
 * kind  = DP_KIND_BOOL / DP_KIND_VALUE (the SDK PROP_BOOL / PROP_VALUE)
 * value = bool as 0/1, value as integer
 * return:  0, handled or explicitly ignored;   -1, unknown dpid
 * 2026.09.21: moved from T1DpMap.cpp by Lff（+AI）
 */
static int T1Tuya_OnObjDp(unsigned char dpid, int kind, int value)
{
    int debug=1;
    char frame[256];
    const T1DpDesc *desc=T1Tuya_DpFind(dpid);

    if ( desc!=NULL && debug>0 ) {
        printf("T1Tuya-OnObjDp: dpid=%u(%s) kind=%d value=%d\n",
               dpid, desc->name, kind, value);
    }

    /* A report-only DP must never be executed when the cloud pushes it */
    if ( desc!=NULL && desc->rw==0 ) {
        if ( debug>0 ) printf("T1Tuya-OnObjDp: dpid=%u is report-only, ignore\n", dpid);
        return 0;
    }

    switch ( dpid ) {
    case DP_ID_BLADE_LIFE:
        /* write 0 resets the counter, a non zero write would set it directly
         * and is not defined in the requirement document yet */
        if ( value==0 ) {
            snprintf(frame, sizeof(frame), "$X:BLADE_RESET");
            T1Tuya_SendToDevice(frame);
        } else if ( debug>0 ) {
            printf("T1Tuya-OnObjDp: dpid=%u set value=%d TODO, no cmd sent\n", dpid, value);
        }
        break;

    case DP_ID_BATTERY_CYCLES:
        /* same convention as the blade life, write 0 resets the counter */
        if ( value==0 ) {
            snprintf(frame, sizeof(frame), "$X:BATTERY_RESET");
            T1Tuya_SendToDevice(frame);
        } else if ( debug>0 ) {
            printf("T1Tuya-OnObjDp: dpid=%u set value=%d TODO, no cmd sent\n", dpid, value);
        }
        break;

    case DP_ID_BLE_SWITCH:
        /* switches the BLE used for near field control, which is a different
         * thing than the BLE netcfg service of the Tuya SDK */
        snprintf(frame, sizeof(frame), "$X:BLE,%d", value ? 1 : 0);
        T1Tuya_SendToDevice(frame);
        break;

    case DP_ID_RAIN_MOW:
    case DP_ID_RAIN_RESUME:
    case DP_ID_RAIN_DELAY:
    case DP_ID_EDGE_DISTANCE:
    case DP_ID_RANDOM_MOW:
    case DP_ID_HEDGEHOG_GUARD:
    case DP_ID_RANDOM_MODE:
        /* TODO: these write a configuration item, mapping it to a device
         * interface (tranCmdStr key / SetStatus field / @CMD line) has to be
         * confirmed with the requirement document first. A wrong guess changes
         * the behaviour of the mower, so nothing is sent until then. */
        if ( debug>0 ) {
            printf("T1Tuya-OnObjDp: dpid=%u mapped TODO, value=%d (no cmd sent)\n", dpid, value);
        }
        break;

    default:
        if ( debug>0 ) printf("T1Tuya-OnObjDp: unknown dpid=%u, ignore\n", dpid);
        return -1;
    }

    return 0;
}

/* cloud -> device: handle one raw DP, mainly the map pushed back from the App
 * dpid = function point id, data = payload, len = byte count
 * return:  0, handled or explicitly ignored;   -1, invalid input
 * 2026.09.21: moved from T1DpMap.cpp by Lff（+AI）
 */
static int T1Tuya_OnRawDp(unsigned char dpid, const unsigned char *data, unsigned len)
{
    int debug=1;

    if ( data==NULL || len==0 ) return -1;

    if ( debug>0 ) printf("T1Tuya-OnRawDp: dpid=%u len=%u\n", dpid, len);

    switch ( dpid ) {
    case DP_ID_LAWN_PROGRESS:
        /* 114 is report only, a downlink is not expected */
        if ( debug>0 ) printf("T1Tuya-OnRawDp: dpid=%u is report-only, ignore\n", dpid);
        break;

    default:
        /* TODO: which DP carries a map push and in which binary format has to
         * be aligned with the App side, for now it is only logged */
        if ( debug>0 ) printf("T1Tuya-OnRawDp: dpid=%u unhandled TODO\n", dpid);
        break;
    }

    return 0;
}

/* ------------------------------------------------------------
 * SDK status callbacks
 * ------------------------------------------------------------ */

/* gateway status changed: reset / activated / first start / normal / ble
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __gw_status_changed_cb(IN CONST GW_STATUS_E status)
{
    switch ( status ) {
    case GW_RESET:       TAL_PR_NOTICE("T1Tuya-GwStatus: reset");       break;
    case GW_ACTIVED:     TAL_PR_NOTICE("T1Tuya-GwStatus: actived");     break;
    case GW_FIRST_START: TAL_PR_NOTICE("T1Tuya-GwStatus: first start"); break;
    case GW_NORMAL:      TAL_PR_NOTICE("T1Tuya-GwStatus: normal");      break;
    case GW_BLE_ACTIVED: TAL_PR_NOTICE("T1Tuya-GwStatus: ble actived"); break;
    default:             TAL_PR_NOTICE("T1Tuya-GwStatus: %d", status);  break;
    }
}

/* gateway reset request, the App removed the device or the button was held
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __gw_reset_cb(IN CONST GW_RESET_TYPE_E type)
{
    TAL_PR_NOTICE("T1Tuya-GwReset: type=%d", type);
}

/* wifi network status, the most useful callback to tell which stage the device
 * is in (netcfg running / connecting / connected)
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __wf_nw_stat_cb(IN CONST GW_WIFI_NW_STAT_E stat)
{
    TAL_PR_NOTICE("T1Tuya-WfNwStat: %d", stat);
}

/* netcfg error code, tells which kind of failure the netcfg run hit
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __wf_cfg_err_code_cb(IN CONST NW_CFG_ERR_CODE_E err_code)
{
    TAL_PR_NOTICE("T1Tuya-WfCfgErr: %d", err_code);
}

/* ============================================================
 * DP121 - move direction (cloud -> device)
 *
 * The value is translated into the same "$C:xx" frame that the 1111 socket and
 * the BLE SPP of the1Comm use, so the device side needs no change at all.
 * ============================================================ */

/* action characters understood by the mower, they are the ones of the "$C:"
 * frame: TheOne takes bufRecv[3] and turns it into "@CMD,%c\r\n" for the F415
 * board, so the action has to be the 4th byte of the frame. */
#define T1_TUYA_MOVE_STOP       'g'     /* DP121 = 0, stop */
#define T1_TUYA_MOVE_FORWARD    'w'     /* DP121 = 1, forward */
#define T1_TUYA_MOVE_BACKWARD   's'     /* DP121 = 2, backward */
#define T1_TUYA_MOVE_LEFT       'a'     /* DP121 = 3, turn left */
#define T1_TUYA_MOVE_RIGHT      'd'     /* DP121 = 4, turn right */

/* translate DP121 and push it down to the mower
 * dir = direction value: 0 stop / 1 forward / 2 backward / 3 left / 4 right
 * return:  0, queued;   -1, downstream not ready or queue full
 * Three conventions worth knowing:
 *   - an out of range value is treated as STOP: when no valid direction can be
 *     decoded the mower must stop, dropping the command would be the unsafe
 *     choice
 *   - the trailing "\r\n" must not be removed, the1Act reads fixed positions
 *   - the frame goes out through T1Tuya_SendToDevice() -> /theone_c2a
 *     (ACTCOMM_CMD_APP) -> the1Act, the very same path the original the1Comm
 *     used for the frames coming from the App
 * 2026.09.21: created by Lff（+AI）
 */
static int T1Tuya_OnMoveDir(int dir)
{
    int debug=1;
    char cmd=T1_TUYA_MOVE_STOP;
    char frame[32]={0};

    switch ( dir ) {
    case 0:  cmd=T1_TUYA_MOVE_STOP;     break;
    case 1:  cmd=T1_TUYA_MOVE_FORWARD;  break;
    case 2:  cmd=T1_TUYA_MOVE_BACKWARD; break;
    case 3:  cmd=T1_TUYA_MOVE_LEFT;     break;
    case 4:  cmd=T1_TUYA_MOVE_RIGHT;    break;
    default:
        if ( debug>0 ) printf("T1Tuya-OnMoveDir: invalid dir=%d, force stop\n", dir);
        cmd=T1_TUYA_MOVE_STOP;
        break;
    }

    snprintf(frame, sizeof(frame), "$C:%c\r\n", cmd);

    if ( T1Tuya_SendToDevice(frame)!=0 ) {
        if ( debug>0 ) printf("T1Tuya-OnMoveDir: send failed, dir=%d\n", dir);
        return -1;
    }

    if ( debug>0 ) printf("T1Tuya-OnMoveDir: dir=%d -> $C:%c\n", dir, cmd);
    return 0;
}

/* ============================================================
 * OTA callbacks
 *
 * FW_UG_S (firmware upgrade info) is passed to every OTA callback:
 *     fw_url    - download address (HTTPS), the SDK fetches it
 *     file_size - size in bytes, used for the disk check and the progress
 *     sw_ver    - target version string, e.g. "1.0.1"
 *     md5/hmac  - checksums of the package
 *     diff_ota  - 1 = differential upgrade
 *     type      - 0 normal (App asks), 1 silent, 2 lan
 * The SDK only tells the application that a new version exists, the download
 * has to be started by the application itself (see __dev_upgrade_inform_cb).
 * ============================================================ */

/* file handle of the package being downloaded, one download at a time */
static FILE *s_otaFp = NULL;

/* pre check before an upgrade starts: refuse when the disk is too small
 * fw = firmware info from the cloud
 * return:  OPRT_OK to allow the upgrade, otherwise a tuya_error_code.h value
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __ota_pre_check(IN CONST FW_UG_S *fw)
{
    struct statfs fs_info;
    UINT64_T need_bytes=0;
    UINT64_T free_bytes=0;
    INT_T ret=OPRT_OK;

    if ( fw==NULL || fw->file_size==0 ) {
        TAL_PR_ERR("T1Tuya-OtaPreCheck: invalid fw info");
        return OPRT_INVALID_PARM;
    }

    memset(&fs_info, 0, sizeof(fs_info));
    if ( access(T1_TUYA_OTA_DIR, F_OK)==0 && statfs(T1_TUYA_OTA_DIR, &fs_info)==0 ) {
        TAL_PR_NOTICE("T1Tuya-OtaPreCheck: disk check on %s", T1_TUYA_OTA_DIR);
    } else if ( statfs("/tmp", &fs_info)==0 ) {
        TAL_PR_NOTICE("T1Tuya-OtaPreCheck: disk check on /tmp");
    } else {
        TAL_PR_ERR("T1Tuya-OtaPreCheck: statfs failed");
        return OPRT_COM_ERROR;
    }

    /* package size plus a 10 MB margin for unpacking */
    need_bytes=(UINT64_T)fw->file_size + 10 * 1024 * 1024;
    free_bytes=(UINT64_T)fs_info.f_bavail * (UINT64_T)fs_info.f_bsize;
    TAL_PR_NOTICE("T1Tuya-OtaPreCheck: need=%llu free=%llu", need_bytes, free_bytes);

    if ( free_bytes<need_bytes ) {
        TAL_PR_ERR("T1Tuya-OtaPreCheck: disk space insufficient, refuse");
        ret=OPRT_COM_ERROR;
    }

    return ret;
}

/* per sub device pre check, this product has no sub device, so it just reuses
 * the gateway check
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __ota_pre_check_dev(IN CONST CHAR_T *dev_id, IN CONST FW_UG_S *fw)
{
    TAL_PR_NOTICE("T1Tuya-OtaPreCheckDev: dev_id=%s", dev_id ? dev_id : "(null)");
    return __ota_pre_check(fw);
}

/* data sink of the download, called by the SDK for every chunk
 * total_len = size of the whole package, offset = position of this chunk
 * data/len  = the chunk itself, remain_len = out, bytes not handled yet (0 here)
 * return:  OPRT_OK when the chunk is stored, otherwise the download is aborted
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static OPERATE_RET __ota_get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len,
                                          IN CONST UINT_T offset, IN CONST BYTE_T *data,
                                          IN CONST UINT_T len, OUT UINT_T *remain_len,
                                          IN PVOID_T pri_data)
{
    size_t written=0;

    if ( remain_len ) *remain_len=0;

    if ( fw==NULL || data==NULL || len==0 ) {
        TAL_PR_WARN("T1Tuya-OtaRecv: invalid param");
        return OPRT_INVALID_PARM;
    }

    if ( s_otaFp==NULL ) {
        mkdir("/app", 0755);
        mkdir("/app/TheOne", 0755);
        mkdir(T1_TUYA_OTA_DIR, 0755);
        unlink(T1_TUYA_OTA_TMP);
        s_otaFp=fopen(T1_TUYA_OTA_TMP, "wb");
        if ( s_otaFp==NULL ) {
            TAL_PR_ERR("T1Tuya-OtaRecv: open tmp failed: %s", strerror(errno));
            return OPRT_COM_ERROR;
        }
        TAL_PR_NOTICE("T1Tuya-OtaRecv: begin, total_len=%u -> %s", total_len, T1_TUYA_OTA_TMP);
    }

    written=fwrite(data, 1, len, s_otaFp);
    if ( written!=len ) {
        TAL_PR_ERR("T1Tuya-OtaRecv: write failed (%u/%u)", (UINT_T)written, len);
        fclose(s_otaFp);
        s_otaFp=NULL;
        unlink(T1_TUYA_OTA_TMP);
        return OPRT_COM_ERROR;
    }

    TAL_PR_NOTICE("T1Tuya-OtaRecv: offset=%u len=%u", offset, len);
    return OPRT_OK;
}

/* download finished (or failed), finish the package and report the result
 * download_result = 0 on success
 * return:  OPRT_OK, the result report itself is best effort
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static OPERATE_RET __ota_upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result,
                                           IN PVOID_T pri_data)
{
    INT_T report_result=TUS_UPGRD_EXEC;
    OPERATE_RET rt=OPRT_OK;

    TAL_PR_NOTICE("T1Tuya-OtaNotify: result=%d", download_result);
    if ( fw ) {
        TAL_PR_NOTICE("T1Tuya-OtaNotify: sw_ver=%s size=%u url=%s",
                      fw->sw_ver, fw->file_size, fw->fw_url);
    }

    if ( s_otaFp ) {
        fclose(s_otaFp);
        s_otaFp=NULL;
    }

    if ( download_result==0 ) {
        if ( rename(T1_TUYA_OTA_TMP, T1_TUYA_OTA_FINAL)!=0 ) {
            TAL_PR_ERR("T1Tuya-OtaNotify: rename failed: %s", strerror(errno));
        } else {
            TAL_PR_NOTICE("T1Tuya-OtaNotify: package ready: %s", T1_TUYA_OTA_FINAL);
            report_result=TUS_UPGRD_FINI;
        }
    } else {
        TAL_PR_ERR("T1Tuya-OtaNotify: download failed, remove tmp");
        unlink(T1_TUYA_OTA_TMP);
    }

    /* without a progress report the App stays on "updating" forever */
    rt=tuya_iot_dev_upgd_progress_rept(100, NULL, DEV_NM_ATH_SNGL);
    TAL_PR_NOTICE("T1Tuya-OtaNotify: report progress 100%% ret=%d", rt);

    rt=tuya_iot_dev_upgd_result_report(NULL, DEV_NM_ATH_SNGL, report_result);
    TAL_PR_NOTICE("T1Tuya-OtaNotify: report result %d ret=%d", report_result, rt);

    return OPRT_OK;
}

/* new firmware announced by the cloud, the download must be started here
 * return:  OPRT_OK when the download was started, otherwise an error code
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __dev_upgrade_inform_cb(CONST FW_UG_S *fw)
{
    OPERATE_RET rt=OPRT_OK;

    if ( fw==NULL ) {
        TAL_PR_ERR("T1Tuya-OtaInform: fw is null");
        return OPRT_INVALID_PARM;
    }

    TAL_PR_NOTICE("================================");
    TAL_PR_NOTICE("T1Tuya-OtaInform: version=%s size=%u diff_ota=%d type=%d",
                  fw->sw_ver, fw->file_size, fw->diff_ota, fw->type);
    TAL_PR_NOTICE("T1Tuya-OtaInform: url=%s", fw->fw_url);
    TAL_PR_NOTICE("================================");

    rt=tuya_iot_upgrade_gw_notify(fw, __ota_get_file_data_cb, __ota_upgrade_notify_cb,
                                  NULL, TRUE, 0);
    if ( rt!=OPRT_OK ) {
        TAL_PR_ERR("T1Tuya-OtaInform: upgrade_gw_notify failed: %d", rt);
        return rt;
    }
    return OPRT_OK;
}

/* pre upgrade hook of the upgrade service, TUS_RD(1) means "go on"
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __pre_dev_upgrade_cb(CONST FW_UG_S *fw)
{
    INT_T ret=__ota_pre_check(fw);
    return (ret==OPRT_OK) ? TUS_RD : ret;
}

/* per sub device upgrade callback, unused on this product
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __dev_ug_cb(IN CONST CHAR_T *dev_id, IN CONST FW_UG_S *fw)
{
    TAL_PR_NOTICE("T1Tuya-DevUgCb: dev_id=%s fw=%p", dev_id ? dev_id : "(null)", fw);
    return OPRT_OK;
}

/* gateway upgrade callback, only logs the version
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static INT_T __gw_ug_cb(IN CONST FW_UG_S *fw)
{
    if ( fw==NULL ) return OPRT_INVALID_PARM;
    TAL_PR_NOTICE("T1Tuya-GwUgCb: version=%s size=%u", fw->sw_ver, fw->file_size);
    return OPRT_OK;
}

/* write the device identity (uuid, auth key, PID) into the SDK key-value store
 * return:  OPRT_OK when the identity is present, otherwise a SDK error code
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static OPERATE_RET __set_auth_info(void)
{
    GW_BASE_IF_S bi;
    OPERATE_RET rt=OPRT_OK;

    memset(&bi, 0, sizeof(bi));
    rt=wd_gw_base_if_read(&bi);
    TAL_PR_NOTICE("T1Tuya-SetAuth: read ret=%d uuid[%s]", rt, bi.uuid);
    if ( rt==OPRT_OK && strlen(bi.uuid)>0 && strlen(bi.auth_key)>0 ) {
        TAL_PR_NOTICE("T1Tuya-SetAuth: already exist, skip");
        return OPRT_OK;
    }

    memset(&bi, 0, sizeof(bi));
    snprintf(bi.uuid,      sizeof(bi.uuid),      "%s", T1_TUYA_UUID);
    snprintf(bi.auth_key,  sizeof(bi.auth_key),  "%s", T1_TUYA_AUTH_KEY);
    snprintf(bi.psk_key,   sizeof(bi.psk_key),   "%s", "");
    snprintf(bi.psk30_key, sizeof(bi.psk30_key), "%s", "");
    snprintf(bi.fac_pin,   sizeof(bi.fac_pin),   "%s", T1_TUYA_PRODUCT_KEY);
    bi.prod_test=FALSE;

    rt=wd_gw_base_if_write(&bi);
    TAL_PR_NOTICE("T1Tuya-SetAuth: write ret=%d", rt);
    return rt;
}

/* settle the netcfg state of this boot: keep a usable store, reset what is not
 * usable any more to "unregistered / not configured" (the state that makes the
 * SDK open the BLE netcfg service again).
 * The reset is deliberately NOT unconditional: forcing it on every boot pushes
 * the device back into netcfg, where the SDK re-activates the wifi profile over
 * and over (measured: 6 drop/connect pairs within one boot). A cloud activation
 * request that lands inside such a drop fails with "fail to get cloud url -2050"
 * and then "activate timeout", so the device stays offline although wlan0 is
 * connected and the App reports "device not found".
 * wsm    = gateway work state from the store: netcfg mode, ssid, passwd, stat
 * bi     = device identity from the store, it carries ap_ssid/ap_passwd as well
 * stat   = UNREGISTERED 0 / REGISTERED 1 / ACTIVATED 2              (gw_intf.h)
 * nc_tp  = GWNS_*: 1/2/10 are still waiting for credentials, 3/4/5/6/7/9/12
 *          already received them                        (tuya_cloud_wifi_defs.h)
 * Force a re-provisioning (new router, changed password, device bound by someone
 * else) by wiping the store and rebooting; the device is then unbound and has to
 * be added in the App again:
 *     sudo rm -f /app/TheOne/tuya_x3m/tuya_user.db* /app/TheOne/tuya_x3m/tuya_enckey.db
 * return:  1, state was reset and written;   0, kept as is (usable store);
 *          -1, a reset was needed but the write failed
 * 2026.09.20: created by Lff（+AI）, add function header comment
 * 2026.09.21: modified by Lff（+AI）, reuse a usable store, repair a state that
 *             lost its credentials, reset only what is really not provisioned
 */
static int __force_unregistered_wsm(void)
{
    GW_WORK_STAT_MAG_S wsm;
    GW_BASE_IF_S       bi;
    OPERATE_RET rt_wsm=OPRT_OK;
    OPERATE_RET rt_bi=OPRT_OK;
    bool identity_ok=false;
    bool cred_ok=false;
    bool provisioned=false;

    memset(&wsm, 0, sizeof(wsm));
    rt_wsm=wd_gw_wsm_read(&wsm);
    if ( rt_wsm!=OPRT_OK ) {
        /* no work state stored yet: a fresh store, treat it as unprovisioned */
        TAL_PR_WARN("T1Tuya-ForceWsm: wsm read failed: %d, treat as unprovisioned", rt_wsm);
        memset(&wsm, 0, sizeof(wsm));
    }

    memset(&bi, 0, sizeof(bi));
    rt_bi=wd_gw_base_if_read(&bi);
    identity_ok=(rt_bi==OPRT_OK && bi.uuid[0]!='\0' && bi.auth_key[0]!='\0');

    /* credentials can sit in the identity record and/or in the work state */
    cred_ok=(bi.ap_ssid[0]!='\0') || (wsm.ssid[0]!='\0');

    /* the GWNS_* values that mean "credentials already received", plus
     * NO_NEED_CFG (wifi does not have to be configured again) */
    provisioned=(wsm.stat!=UNREGISTERED) ||
                (wsm.nc_tp==GWNS_TY_SMARTCFG)    || (wsm.nc_tp==GWNS_TY_AP) ||
                (wsm.nc_tp==GWNS_WECHAT_AK)      || (wsm.nc_tp==GWNS_OTHER_CFG) ||
                (wsm.nc_tp==GWNS_NO_NEED_CFG)    || (wsm.nc_tp==GWNS_TY_SMART_AP_CFG) ||
                (wsm.nc_tp==GWNS_PROXY_CONFIG_WF);

    if ( identity_ok && cred_ok && provisioned ) {
        TAL_PR_NOTICE("T1Tuya-ForceWsm: usable store, keep it (stat=%d nc_tp=%d ssid=%s)",
                      wsm.stat, wsm.nc_tp, wsm.ssid[0]!='\0' ? wsm.ssid : bi.ap_ssid);
        return 0;
    }

    /* credentials are there but the work state lost them (for example a store
     * that was written halfway): repair the state from the identity record
     * instead of asking for a new provisioning run */
    if ( identity_ok && cred_ok ) {
        snprintf(wsm.ssid,   sizeof(wsm.ssid),   "%s", bi.ap_ssid);
        snprintf(wsm.passwd, sizeof(wsm.passwd), "%s", bi.ap_passwd);
        wsm.nc_tp=GWNS_NO_NEED_CFG;
        wsm.md=GWM_NORMAL;
        wsm.stat=REGISTERED;
        rt_wsm=wd_gw_wsm_write(&wsm);
        TAL_PR_NOTICE("T1Tuya-ForceWsm: state repaired from the identity record, write ret=%d", rt_wsm);
        if ( rt_wsm==OPRT_OK ) return 0;
    }

    TAL_PR_NOTICE("T1Tuya-ForceWsm: reset to unprovisioned (identity=%d cred=%d stat=%d nc_tp=%d)",
                  identity_ok ? 1 : 0, cred_ok ? 1 : 0, wsm.stat, wsm.nc_tp);

    wsm.nc_tp=GWNS_UNCFG_SMC;
    wsm.md=GWM_SPECIAL_SMT_CFG;
    wsm.stat=UNREGISTERED;
    memset(wsm.ssid,   0, sizeof(wsm.ssid));
    memset(wsm.passwd, 0, sizeof(wsm.passwd));

    rt_wsm=wd_gw_wsm_write(&wsm);
    TAL_PR_NOTICE("T1Tuya-ForceWsm: write ret=%d stat=%d", rt_wsm, wsm.stat);

    return (rt_wsm==OPRT_OK) ? 1 : -1;
}

/* ============================================================
 * Log redirection
 *
 * The Tuya SDK prints a lot and does it from its own threads. All of it,
 * together with our own printf, is redirected into /tmp/tuya.log so that the
 * stdout of the1Comm (tee'ed into /tmp/the1.log) stays readable.
 *
 * Three file descriptors are involved for one file:
 *     fd     what the process uses, "the 3rd slot of my table"
 *     OFD    the open instance, holds the file offset and the open flags
 *     inode  the file itself, pure metadata, unrelated to "opening"
 * A dup() only copies the fd, both fds then share the same OFD, hence the same
 * offset. That is why the original fd is saved with dup() before dup2() and
 * restored the same way: the offset and the buffering state stay consistent.
 * close(1) followed by dup(fd) is not used because it leaves a window where
 * another thread could grab fd 1.
 * ============================================================ */

#define T1_TUYA_LOG_FILE          "/tmp/tuya.log"
#define T1_TUYA_LOG_ROTATE_BYTES  (4 * 1024 * 1024)

static int  g_logSavedOut   = -1;       /* copy of the original stdout, -1 = none */
static int  g_logSavedErr   = -1;       /* copy of the original stderr, -1 = none */
static bool g_logRedirected = false;    /* true = stdout/stderr currently point at the log */

/* rotate the log once it grows past T1_TUYA_LOG_ROTATE_BYTES, keeping one
 * generation as .old so that a full partition is impossible
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static void T1Tuya_LogRotate(void)
{
    int debug=1;
    struct stat st;
    char old[160];

    if ( stat(T1_TUYA_LOG_FILE, &st)!=0 ) return;
    if ( st.st_size<=T1_TUYA_LOG_ROTATE_BYTES ) return;

    snprintf(old, sizeof(old), "%s.old", T1_TUYA_LOG_FILE);
    unlink(old);
    if ( rename(T1_TUYA_LOG_FILE, old)!=0 ) {
        if ( debug>0 ) printf("T1Tuya-LogRotate: rename %s failed: %s\n",
                              T1_TUYA_LOG_FILE, strerror(errno));
    }
}

/* point stdout and stderr at the log file, keeping copies of the originals
 * path = log file path
 * return:  0, redirected (also when it was already done);   -1, failure, io unchanged
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static int T1Tuya_LogRedirect(const char *path)
{
    int debug=1;
    int fd=-1;

    if ( path==NULL || path[0]=='\0' ) return -1;
    if ( g_logRedirected ) return 0;

    T1Tuya_LogRotate();

    fd=open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if ( fd<0 ) {
        if ( debug>0 ) printf("T1Tuya-LogRedirect: open(%s) failed: %s, keep stdout\n",
                              path, strerror(errno));
        return -1;
    }

    /* everything buffered so far belongs to the old destination */
    fflush(stdout);
    fflush(stderr);

    /* dup() gives an fd on the same OFD, so the offset survives the restore */
    g_logSavedOut=dup(STDOUT_FILENO);
    g_logSavedErr=dup(STDERR_FILENO);

    if ( dup2(fd, STDOUT_FILENO)<0 || dup2(fd, STDERR_FILENO)<0 ) {
        if ( g_logSavedOut>=0 ) {
            dup2(g_logSavedOut, STDOUT_FILENO);
            close(g_logSavedOut);
            g_logSavedOut=-1;
        }
        if ( g_logSavedErr>=0 ) {
            dup2(g_logSavedErr, STDERR_FILENO);
            close(g_logSavedErr);
            g_logSavedErr=-1;
        }
        close(fd);
        if ( debug>0 ) printf("T1Tuya-LogRedirect: dup2 failed: %s, keep stdout\n", strerror(errno));
        return -1;
    }

    close(fd);
    g_logRedirected=true;
    return 0;
}

/* restore the original stdout/stderr, used when the channel shuts down so that
 * the final messages are visible again
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static void T1Tuya_LogRestore(void)
{
    if ( !g_logRedirected ) return;

    fflush(stdout);
    fflush(stderr);

    if ( g_logSavedOut>=0 ) {
        dup2(g_logSavedOut, STDOUT_FILENO);
        close(g_logSavedOut);
        g_logSavedOut=-1;
    }
    if ( g_logSavedErr>=0 ) {
        dup2(g_logSavedErr, STDERR_FILENO);
        close(g_logSavedErr);
        g_logSavedErr=-1;
    }
    g_logRedirected=false;
}

/* constructor, only sets the state, no SDK and no hardware access
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
T1Tuya::T1Tuya() : m_ready(false)
{
}

/* destructor, closes the OTA staging file, the device endpoint and the log
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
T1Tuya::~T1Tuya()
{
    if ( s_otaFp ) {
        fclose(s_otaFp);
        s_otaFp=NULL;
    }
    if ( s_actOut!=(mqd_t)-1 ) {
        mq_close(s_actOut);
        s_actOut=(mqd_t)-1;
    }
    T1Tuya_LogRestore();
}

/* cloud -> device: handle one downlink of ordinary DP (bool / value / enum)
 * dp = TY_RECV_OBJ_DP_S, dp->dps[] holds one entry per DP of this downlink
 * return: none, the internal handlers own the per DP result
 * 2026.09.20: created by Lff（+AI）, add function header comment
 * 2026.09.21: modified by Lff（+AI）, intercept DP121 before the DP table
 */
static VOID_T __dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    int debug=1;
    UINT_T i=0;

    if ( dp==NULL ) return;
    if ( debug>0 ) printf("T1Tuya-OnObjDp: cnt=%u\n", dp->dps_cnt);

    for ( i=0; i<dp->dps_cnt; i++ ) {
        const TY_OBJ_DP_S *one=&dp->dps[i];
        int value=0;

        /* flatten the union to a single int, the SDK type decides which field
         * carries the value, a string DP is not used by this product */
        if ( one->type==PROP_BOOL )       value=one->value.dp_bool ? 1 : 0;
        else if ( one->type==PROP_VALUE ) value=one->value.dp_value;
        else if ( one->type==PROP_ENUM )  value=(int)one->value.dp_enum;
        else                              value=0;

        if ( debug>0 ) {
            printf("T1Tuya-OnObjDp: dpid=%u type=%d value=%d\n", one->dpid, one->type, value);
        }

        /* DP121 is the downlink of the channel itself (direction -> "$C:"),
         * it is handled here instead of in the DP table, the DP table covers
         * the parameter DPs of the requirement document */
        if ( one->dpid==DP_ID_MOVE_DIR ) {
            T1Tuya_OnMoveDir(value);
            continue;
        }

        T1Tuya_OnObjDp(one->dpid, one->type, value);
    }
}

/* cloud -> device: handle one raw DP downlink (map content, big blocks)
 * dp = TY_RECV_RAW_DP_S, dp->data is NOT NUL terminated, always use dp->len
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    int debug=1;
    if ( dp==NULL ) return;
    if ( debug>0 ) printf("T1Tuya-OnRawDp: dpid=%u len=%u\n", dp->dpid, dp->len);
    T1Tuya_OnRawDp(dp->dpid, dp->data, dp->len);
}

/* the App asks for the current value of some DP (when it opens or refreshes)
 * dp_qry = TY_DP_QUERY_S, cnt == 0 means "all of them" and dpid[] is empty
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
static VOID_T __dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)
{
    int debug=1;
    UINT_T i=0;

    if ( dp_qry==NULL ) return;
    if ( debug>0 ) printf("T1Tuya-OnDpQuery: cnt=%u\n", dp_qry->cnt);
    for ( i=0; i<dp_qry->cnt; i++ ) {
        if ( debug>0 ) printf("T1Tuya-OnDpQuery: want dpid=%u\n", dp_qry->dpid[i]);
    }
    /* TODO: answer with T1Tuya_ReportDp*() using the real device state read
     * from the1Act. Until the frame layout is aligned nothing is answered, a
     * made up value would be worse than a missing one. */
}

/* bring up the Tuya SDK: directories, identity, BLE netcfg, cloud link and the
 * DP / OTA callbacks. Every SDK call of the channel is issued from here.
 * Call order matters: this function must be called AFTER MSG_COMM_READY was
 * sent to the1, its WaitForReady() timeout is 5 s (T1tri/T1IPCMain2Comm.cpp:30)
 * and a timeout makes the1 execv("../the1Update", "--rollback").
 * return:  0, channel ready (IsReady() becomes true);   -1, init failed
 * 2026.09.20: created by Lff（+AI）, add function header comment
 * 2026.09.21: modified by Lff（+AI）, the device endpoint is now part of this file
 */
int T1Tuya::Init()
{
    int debug=1;
    OPERATE_RET rt=OPRT_OK;

    m_ready=false;

    /* 1. our own messages go to the log file from here on */
    printf("T1Tuya-Init: redirecting Tuya logs to %s\n", T1_TUYA_LOG_FILE);
    if ( T1Tuya_LogRedirect(T1_TUYA_LOG_FILE)==0 ) {
        printf("========== Tuya log start, pid=%d ==========\n", (int)getpid());
    }

    tuya_iot_set_log_attr(TAL_LOG_LEVEL_NOTICE);

    /* 2. working directories, mkdir() simply fails with EEXIST when present */
    mkdir("/app",          0755);
    mkdir("/app/TheOne",   0755);
    mkdir("/userdata",     0755);
    mkdir("/userdata/app", 0755);

    if ( T1Tuya_EnsureDirWritable(T1_TUYA_STORAGE_PATH, 0755)!=0 ) {
        printf("T1Tuya-Init: storage dir %s unusable, fallback to %s\n",
               T1_TUYA_STORAGE_PATH, T1_TUYA_FALLBACK_BASE);
        if ( T1Tuya_EnsureDirWritable(T1_TUYA_FALLBACK_BASE, 0755)!=0 ) {
            printf("T1Tuya-Init: fallback %s also unusable, abort "
                   "(the Tuya channel stays down, IPC keeps running)\n",
                   T1_TUYA_FALLBACK_BASE);
            return -1;
        }
        snprintf(s_storagePath, sizeof(s_storagePath), "%s", T1_TUYA_FALLBACK_BASE);
    } else {
        snprintf(s_storagePath, sizeof(s_storagePath), "%s", T1_TUYA_STORAGE_PATH);
    }

    snprintf(s_logPath, sizeof(s_logPath), "%slog/", s_storagePath);
    if ( T1Tuya_EnsureDirWritable(s_logPath, 0755)!=0 ) {
        printf("T1Tuya-Init: log dir %s unusable, run without log_seq_path\n", s_logPath);
        s_logPath[0]='\0';
    }

    if ( T1Tuya_EnsureDirWritable(T1_TUYA_OTA_DIR, 0755)!=0 ) {
        printf("T1Tuya-Init: OTA dir %s NOT writable, "
               "cloud OTA download will fail later\n", T1_TUYA_OTA_DIR);
    }

    printf("T1Tuya-Init: storage=%s log=%s\n",
           s_storagePath, (s_logPath[0]!='\0') ? s_logPath : "(none)");

    /* 3. device side endpoint: open the write end of /theone_c2a used for
     *    "cloud -> device". A failure here is not fatal, the queue is opened
     *    lazily again on the first downlink (the1Act may not be up yet). */
    T1Tuya_EnsureActOut();

    /* 4. BLE is kept open for netcfg, that is the only way to pair a device
     *    that has no keyboard and no display */
    tuya_ble_set_serv_switch(TRUE);
    tuya_ble_set_startup_attr(TUYA_BLE_ABILITY_NETCFG);
    tuya_enable_ble_netcfg(TRUE);

    /* 5. base services of the SDK: KV database, logs, system layer. This is
     *    the foundation of everything below and the step that fails first when
     *    the storage path is not writable. */
    /* 5a. store health check - MUST run before the SDK opens the store below:
     *     complete store -> reused as is; broken -> restored from the SDK backup;
     *     beyond repair -> wiped, a fresh store is built and the device has to be
     *     provisioned again. */
    T1Tuya_KvStoreCheck();

    TY_INIT_PARAMS_S init_param;
    memset(&init_param, 0, sizeof(init_param));
    init_param.init_db=TRUE;
    snprintf(init_param.sys_env,      SYS_ENV_LEN,       "%s", TARGET_PLATFORM);
    if ( s_logPath[0]!='\0' ) {
        snprintf(init_param.log_seq_path, LOG_SEQ_PATH_LEN, "%s", s_logPath);
    }

    rt=tuya_iot_init_params(s_storagePath, &init_param);
    if ( rt!=OPRT_OK ) {
        TAL_PR_ERR("T1Tuya-Init: tuya_iot_init_params failed: %d (storage=%s)", rt, s_storagePath);
        return -1;
    }
    TAL_PR_NOTICE("T1Tuya-Init: tuya iot init ok");

    /* 6. device identity, written once and reused on every later boot */
    if ( __set_auth_info()!=OPRT_OK ) {
        TAL_PR_WARN("T1Tuya-Init: set auth info failed, continue anyway");
    }

    /* 7. the two "observer" callbacks are registered on their own interfaces
     *    and are not part of the callback table of step 8 */
    tuya_iot_reg_get_wf_nw_stat_cb(__wf_nw_stat_cb);
    tuya_iot_set_wf_cfg_err_code_cb(__wf_cfg_err_code_cb);

    /* 8. callback table: DP downlink, OTA hooks and the gateway status */
    TY_IOT_CBS_S iot_cbs;
    memset(&iot_cbs, 0, sizeof(iot_cbs));
    iot_cbs.gw_status_cb    = __gw_status_changed_cb;
    iot_cbs.gw_ug_cb        = __gw_ug_cb;
    iot_cbs.gw_reset_cb     = __gw_reset_cb;
    iot_cbs.dev_obj_dp_cb   = __dev_obj_dp_cb;
    iot_cbs.dev_raw_dp_cb   = __dev_raw_dp_cb;
    iot_cbs.dev_dp_query_cb = __dev_dp_query_cb;
    iot_cbs.dev_ug_cb       = __dev_ug_cb;
    iot_cbs.pre_gw_ug_cb    = __ota_pre_check;
    iot_cbs.pre_dev_ug_cb   = __ota_pre_check_dev;

    if ( debug>0 ) {
        printf("T1Tuya-Init: iot_cbs gw_ug_cb=%p dev_ug_cb=%p pre_dev_ug_cb=%p\n",
               iot_cbs.gw_ug_cb, iot_cbs.dev_ug_cb, iot_cbs.pre_dev_ug_cb);
    }

    /* 9. netcfg state: a usable store keeps its ssid/passwd so the device goes
     *    straight to the cloud link, only what is not usable any more is reset
     *    to "unregistered / not configured" (which is what opens the BLE netcfg
     *    service again). Force a re-provisioning by wiping the store, then
     *    reboot; the device has to be added in the App again afterwards:
     *        sudo rm -f /app/TheOne/tuya_x3m/tuya_user.db* /app/TheOne/tuya_x3m/tuya_enckey.db */
    __force_unregistered_wsm();

    /* 10. persist the netcfg start mode so that the next boot does not depend
     *     on this call, then hand the wifi parameters to the SDK */
    wifi_save_start_mode(WF_START_SMART_ONLY);
    TUYA_WIFI_CFG_PARAM wifi_cfg_param;
    memset(&wifi_cfg_param, 0, sizeof(wifi_cfg_param));
    wifi_cfg_param.mthd=GWCM_OLD;
    wifi_cfg_param.start_mode=WF_START_SMART_ONLY;
    wifi_cfg_param.enable_station_ap=FALSE;
    set_wifi_config_params(wifi_cfg_param);

    /* 11. the point of no return: "this process is the SOC device itself".
     *     From here on the SDK owns the wifi interface and the cloud link. */
    rt=tuya_iot_wf_soc_dev_init_param(GWCM_OLD, WF_START_SMART_ONLY, &iot_cbs,
                                      (CHAR_T *)T1_TUYA_FIRMWARE_KEY,
                                      (CHAR_T *)T1_TUYA_PRODUCT_KEY,
                                      (CHAR_T *)T1_TUYA_SW_VER);
    if ( rt!=OPRT_OK ) {
        TAL_PR_ERR("T1Tuya-Init: wf_soc_dev_init_param failed: %d", rt);
        return -1;
    }
    TAL_PR_NOTICE("T1Tuya-Init: wifi soc dev init ok");

    /* 12. upgrade service, the download callbacks are handed over here */
    TUYA_UPGRADE_PARAM_T upgrade_param;
    memset(&upgrade_param, 0, sizeof(upgrade_param));
    upgrade_param.dev_upgrade_cb     = __dev_upgrade_inform_cb;
    upgrade_param.subdev_upgrade_cb  = NULL;
    upgrade_param.pre_dev_upgrade_cb = __pre_dev_upgrade_cb;

    rt=tuya_svc_upgrade_init(&upgrade_param);
    if ( rt!=OPRT_OK ) {
        TAL_PR_ERR("T1Tuya-Init: tuya_svc_upgrade_init failed: %d", rt);
        return -1;
    }

    /* 13. self check and the switch that lets the event loop fan out to us */
    T1Tuya_DumpDpTable();

    m_ready=true;
    if ( debug>0 ) {
        printf("T1Tuya-Init: ok, ble serv stat=%d (0=OPENED,1=OPENING,2=CLOSED,3=CLOSING)\n",
               tuya_ble_get_serv_stat());
    }
    return 0;
}

/* drive one round of the SDK, called by the event loop on every iteration
 * HARD REQUIREMENT: must not block. This call shares the loop with the IPC
 * reads and writes, a block here freezes the1 and the1Act communication.
 * return:  always 0
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
int T1Tuya::Select()
{
    int debug=1;
    static int tick=0;

    /* the BLE service state is worth a line every 500 rounds: it is the
     * quickest hint on whether netcfg is still running */
    if ( ++tick>=500 ) {
        tick=0;
        if ( debug>0 ) {
            printf("T1Tuya-Select: ble serv stat=%d (0=OPENED,1=OPENING,2=CLOSED,3=CLOSING)\n",
                   tuya_ble_get_serv_stat());
        }
    }
    return 0;
}

/* handle one inbound round of "cloud -> device"
 * The SDK delivers the DP through __dev_obj_dp_cb / __dev_raw_dp_cb from its
 * own thread, so there is nothing to dequeue here. Kept because the event loop
 * of T1CommMain.cpp calls it (same interface as the other channels).
 * return:  always 0
 * 2026.09.20: created by Lff（+AI）, add function header comment
 */
int T1Tuya::RecvData()
{
    return 0;
}

/* extract the status value out of a the1Act heart beat frame
 * frame = NUL terminated frame, e.g.
 *         ssid:The1One100666V:0.0.0Z:0.0.0P:0,0,0F:-1M:17534N:0S:3B:88\n
 *         built by T1All::getHeartBeatStr() on the the1Act side
 * return:  0..4, device status (0 idle / 1 out of dock / 2 back to dock /
 *          3 mowing / 4 paused);   -1, "S:" not found or value out of range
 * 2026.09.21: created by Lff（+AI）
 */
static int T1Tuya_ParseStatusField(const char *frame)
{
    int debug=0;
    const char *p=NULL;
    int status=0;

    if ( frame==NULL ) return -1;

    /* "S:" must be followed by a digit. A plain strstr() is not enough here:
     * the ssid at the head of the frame is read at runtime (T1Utils::meSsid),
     * an ssid containing "S:" (e.g. "MyS:Home") would match it. Together with
     * the 0..4 check below this also keeps the other fields (V:/Z:/P:/F:/M:/
     * N:/B:) out of the way. */
    p=strstr(frame, "S:");
    while ( p!=NULL ) {
        if ( p[2]>='0' && p[2]<='9' ) break;
        p=strstr(p+2, "S:");
    }
    if ( p==NULL ) return -1;

    status=atoi(p+2);
    if ( status<0 || status>4 ) {
        if ( debug>0 ) printf("T1Tuya-ParseStatus: out of range, status=%d frame=[%.160s]\n", status, frame);
        return -1;
    }
    return status;
}

/* report DP113 (device status) to the cloud, skipped when nothing changed
 * status = 0..4, the same values JudgeWorkStatus() returns on the the1Act side
 * return:  0, reported or skipped because unchanged;   -1, out of range or the
 *          report call failed
 * lastStatus starts at -1 on purpose: the first status of a boot is then
 * reported even when it is 0 (idle), otherwise the App would show an empty
 * status until the mower changes state.
 * 2026.09.21: created by Lff（+AI）
 */
static int T1Tuya_ReportDeviceStatus(int status)
{
    int debug=1;
    static int lastStatus=-1;

    if ( status<0 || status>4 ) {
        if ( debug>0 ) printf("T1Tuya-ReportDevStatus: invalid status=%d, drop\n", status);
        return -1;
    }

    if ( status==lastStatus ) return 0;
    lastStatus=status;

    if ( debug>0 ) printf("T1Tuya-ReportDevStatus: status=%d\n", status);

    return T1Tuya_ReportDpValue(DP_ID_DEVICE_STATUS, status);
}

/* push one frame produced on the device side up to the Tuya cloud
 * data = NUL terminated text frame, three formats are handled in this order:
 *          1. "$D:<dpid>,<type>,<value>"   debug report, type 0 = bool, 1 = value
 *          2. heart beat frame of the1Act, its "S:" field becomes DP113
 *          3. anything else ($X: / $C: ...), logged only, never reported
 * return:  0, handled or deliberately ignored;   -1, bad argument or the
 *          channel is not ready and the frame was dropped
 * The device -> cloud direction is driven by the event loop of T1CommMain.cpp
 * (it feeds every frame of the1 / the1Act into this call, exactly like the
 * original the1Comm fanned its frames out to blueT / socket / mqtt).
 * 2026.09.20: created by Lff（+AI）, add function header comment
 * 2026.09.21: modified by Lff（+AI）, heart beat frame -> DP113 report
 */
int T1Tuya::SendData(const char *data)
{
    int debug=1;
    int dpid=0;
    int type=0;
    int value=0;
    int status=0;

    if ( data==NULL ) return -1;
    if ( !m_ready ) {
        if ( debug>0 ) printf("T1Tuya-SendData: channel not ready, drop\n");
        return -1;
    }

    /* format 1: debug report  $D:<dpid>,<type>,<value>
     * e.g. $D:113,1,3 reports DP113 status = 3 */
    if ( sscanf(data, "$D:%d,%d,%d", &dpid, &type, &value)==3 ) {
        if ( dpid<0 || dpid>255 ) return -1;
        if ( type==0 ) {
            T1Tuya_ReportDpBool((unsigned char)dpid, value);
        } else {
            T1Tuya_ReportDpValue((unsigned char)dpid, value);
        }
        if ( debug>0 ) {
            printf("T1Tuya-SendData: reported dpid=%d type=%d value=%d\n", dpid, type, value);
        }
        return 0;
    }

    /* format 2: heart beat frame of the1Act, the uplink main path. The status
     * of the mower sits in its "S:" field, it becomes DP113 (dedup inside). */
    status=T1Tuya_ParseStatusField(data);
    if ( status>=0 ) {
        T1Tuya_ReportDeviceStatus(status);
        return (int)strlen(data);
    }

    /* format 3: everything else. Only logged until the mapping rules are
     * aligned with the the1Act frame layout, nothing is reported on a guess. */
    if ( debug>0 ) printf("T1Tuya-SendData: unhandled frame: %s\n", data);
    return 0;
}
