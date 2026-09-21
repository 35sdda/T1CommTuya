#ifndef T1TUYA_HPP
#define T1TUYA_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * T1Tuya - Tuya channel of the1Comm
 *
 * The interface is frozen. These five signatures are called by the event loop
 * of T1CommMain.cpp and are the same ones the original the1Comm expects, so
 * the module can be dropped into that repository without touching it:
 *     tuya.Init();                        // T1CommMain.cpp:290
 *     if (tuya.IsReady()) tuya.SendData() // T1CommMain.cpp:105 / :135
 *     tuya.Select(); tuya.RecvData();     // T1CommMain.cpp:214-216
 *
 * Everything else of the channel lives in T1Tuya.cpp - the DP table, the
 * downlink translation and the mqueue used to reach the1Act. This header
 * stays the only interface of the module, no further file is involved.
 *
 * 2026.09.20: created by Lff（+AI）
 * 2026.09.21: modified by Lff（+AI）, DP table and device endpoint moved into
 *             T1Tuya.cpp
 * ============================================================ */
class T1Tuya {
public:
    /* constructor and destructor only handle state, no hardware, no SDK
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    T1Tuya();

    /* release what the channel owns: the OTA staging file, the write end of
     * the mqueue towards the1Act and the log redirection
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    ~T1Tuya();

    /* bring up the Tuya SDK: working directories, device identity, BLE netcfg,
     * cloud link and the DP / OTA callbacks. The outer event loop calls it
     * exactly once.
     * Call it AFTER MSG_COMM_READY was sent to the1: WaitForReady() of the1
     * only waits 5 s (T1tri/T1IPCMain2Comm.cpp:30) and a timeout rolls the
     * whole system back.
     * return:  0, SDK ready (IsReady() becomes true);   -1, init failed and the
     *          channel stays bypassed
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    int Init();

    /* drive the SDK for one round, called by the event loop every iteration
     * MUST NOT block: the loop shares this call with the IPC of the1/the1Act,
     * blocking here freezes their communication.
     * return:  always 0
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    int Select();

    /* take one inbound round of "cloud -> device"; the SDK DP callbacks do the
     * work, nothing is queued here
     * return:  always 0
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    int RecvData();

    /* push one frame produced on the device side up to the Tuya cloud
     * data = NUL terminated text frame, following the existing "$X:" / "$C:"
     * convention of the device side
     * return:  0, handed over;   -1, bad argument or the channel is not ready
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    int SendData(const char *data);

    /* tells the event loop whether frames may be fanned out to this channel
     * return:  true, SDK up;   false, not ready
     * 2026.09.20: created by Lff（+AI）, add function header comment
     */
    bool IsReady() const { return m_ready; }

private:
    bool m_ready;       /* true after a successful Init(), false before */

    /* copying would duplicate the global SDK state, i.e. two "devices" */
    T1Tuya(const T1Tuya &);
    T1Tuya &operator=(const T1Tuya &);
};

/* ============================================================
 * device -> cloud: DP report helpers
 * They wrap dev_report_dp_json_async / dev_report_dp_raw_sync_extend. Basic C
 * types are used on purpose, so that the SDK types do not leak into this
 * header. The dpid values are listed in the DP table section of T1Tuya.cpp.
 * return: OPRT_OK on success, otherwise a tuya_error_code.h value
 * 2026.09.20: created by Lff（+AI）, add function header comment
 * ============================================================ */
int T1Tuya_ReportDpBool(unsigned char dpid, int value);                 /* value: 0/1 */
int T1Tuya_ReportDpValue(unsigned char dpid, int value);
int T1Tuya_ReportDpRaw(unsigned char dpid, const unsigned char *data, unsigned len);

#endif
