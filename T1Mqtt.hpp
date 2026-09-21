#ifndef T1MQTT_HPP
#define T1MQTT_HPP

/* ============================================================
 * BUILD SHIM - this file belongs to the Tuya staging app only.
 *
 * T1CommMain.cpp is carried over verbatim from the1Comm and includes this
 * header, but the MQTT channel is not part of this build: the real T1Mqtt
 * needs <mosquitto.h> and -lmosquitto -lcurl, and neither is part of the Tuya
 * SDK build. This no-op stand-in keeps the original main compilable, and it is
 * the reason why this app links without mosquitto and curl at all.
 *
 * Do NOT copy this file back into the1Comm: the repository has the real
 * T1Mqtt.hpp plus T1Mqtt.cpp, and the broker settings live with them.
 *
 * IsReady() is always false, so every MQTT branch of the event loop is
 * bypassed and the channel that really runs is the Tuya one.
 * 2026.09.21: created by Lff（+AI）, build shim for the Tuya staging app
 * ============================================================ */

#include <stdio.h>
#include <string>

class T1Mqtt {
public:
    T1Mqtt() {}
    ~T1Mqtt() {}

    /* broker = address, port = port, topic = subscribe topic, username and
     * password = login of the broker, NULL for anonymous
     * return: always -1, the channel is not available in this build
     */
    int Init(const char *broker, int port, const char *topic,
             const char *username, const char *password) {
        (void)broker; (void)port; (void)topic; (void)username; (void)password;
        return -1;
    }

    /* topic used when the device pushes data
     * 2026.09.21: created by Lff（+AI）, build shim
     */
    void SetPubTopic(const char *topic) { (void)topic; }

    /* drive the network, kept so that the event loop keeps its shape */
    int Select() { return 0; }

    /* push one message, return: always -1, not available */
    int SendData(const char *data) { (void)data; return -1; }

    /* return: always false, the event loop skips the channel */
    bool IsReady() const { return false; }
};

#endif
