#include "cameraAi.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

#include "appState.h"
#include "oledInit.h"
#include "servoControl.h"

namespace
{
    // 调试输出控制：设为 true 可打印详细的 UART 接收和状态检测信息
    static constexpr bool CAM_DEBUG_VERBOSE = true;
    // 周期性状态报告间隔（毫秒）
    static constexpr uint32_t CAM_STATUS_REPORT_INTERVAL_MS = 5000;
    static uint32_t s_lastStatusReportMs = 0;

    void trimLabelInPlace(char *s)
    {
        if (s == nullptr || s[0] == '\0')
        {
            return;
        }
        size_t n = strlen(s);
        while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r'))
        {
            s[--n] = '\0';
        }
    }

    bool labelEqualsCi(const char *a, const char *b)
    {
        if (a == nullptr || b == nullptr)
        {
            return false;
        }
        while (*a != '\0' && *b != '\0')
        {
            const unsigned char ca = static_cast<unsigned char>(*a++);
            const unsigned char cb = static_cast<unsigned char>(*b++);
            if (std::tolower(ca) != std::tolower(cb))
            {
                return false;
            }
        }
        return *a == *b;
    }

    void setAiState(bool detected, const char *label, float conf)
    {
        g_lastAiDetected = detected;
        g_lastAiConf = conf;
        g_lastAiUpdateMs = millis();

        strncpy(g_lastAiLabel, label, sizeof(g_lastAiLabel) - 1);
        g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';

        setAiResult(detected, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs);
    }

    void clearAiOfflineFlag()
    {
        g_aiOfflineReported = false;
    }

    void checkDeviceAndServiceStatus(uint32_t now)
    {
        // 重新获取当前时间，避免与 parseCameraLine 中更新的时间戳不一致
        // （loop 开始时的 now 可能比 pollCameraUart 中更新的 g_lastCamHeartbeatMs 旧）
        now = millis();

        const bool heartbeatMissing =
            (g_lastCamHeartbeatMs == 0 || (now - g_lastCamHeartbeatMs) > CAM_HEARTBEAT_TIMEOUT_MS);

        // 周期性状态报告
        if (CAM_DEBUG_VERBOSE && (now - s_lastStatusReportMs) >= CAM_STATUS_REPORT_INTERVAL_MS)
        {
            s_lastStatusReportMs = now;
            Serial.printf("[CAM-DBG] === STATUS REPORT ===\n");
            Serial.printf("[CAM-DBG] now=%lu, lastHeartbeat=%lu, diff=%lu ms\n",
                          now, g_lastCamHeartbeatMs,
                          g_lastCamHeartbeatMs > 0 ? (now - g_lastCamHeartbeatMs) : 0);
            Serial.printf("[CAM-DBG] lastAiSuccess=%lu, diff=%lu ms\n",
                          g_lastAiSuccessMs,
                          g_lastAiSuccessMs > 0 ? (now - g_lastAiSuccessMs) : 0);
            Serial.printf("[CAM-DBG] camReady=%d, aiOfflineReported=%d\n",
                          g_camReady ? 1 : 0, g_aiOfflineReported ? 1 : 0);
            Serial.printf("[CAM-DBG] heartbeatMissing=%d (timeout=%lu ms)\n",
                          heartbeatMissing ? 1 : 0, CAM_HEARTBEAT_TIMEOUT_MS);
        }

        if (heartbeatMissing || !g_camReady)
        {
            if (CAM_DEBUG_VERBOSE)
            {
                Serial.printf("[CAM-DBG] -> CAM_OFFLINE: heartbeatMissing=%d, camReady=%d\n",
                              heartbeatMissing ? 1 : 0, g_camReady ? 1 : 0);
            }
            setAiError(true, now, AI_ERR_CAM_OFFLINE);
            return;
        }

        const bool aiTimedOut =
            (g_lastAiSuccessMs == 0 || (now - g_lastAiSuccessMs) > AI_OFFLINE_TIMEOUT_MS);
        if (g_aiOfflineReported || aiTimedOut)
        {
            if (CAM_DEBUG_VERBOSE)
            {
                Serial.printf("[CAM-DBG] -> AI_OFFLINE: aiOfflineReported=%d, aiTimedOut=%d\n",
                              g_aiOfflineReported ? 1 : 0, aiTimedOut ? 1 : 0);
            }
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE);
            return;
        }

        setAiError(false, now);
    }

    void parseCameraLine(const char *line)
    {
        if (line == nullptr || line[0] == '\0')
        {
            return;
        }

        while (*line == ' ' || *line == '\t')
        {
            ++line;
        }

        // 调试：打印收到的每行数据
        if (CAM_DEBUG_VERBOSE)
        {
            Serial.printf("[CAM-DBG] RX LINE: \"%s\" (len=%d)\n", line, strlen(line));
        }

        uint32_t now = millis();
        g_lastCamHeartbeatMs = now;

        if (strcmp(line, "READY") == 0)
        {
            g_camReady = true;
            if (CAM_DEBUG_VERBOSE)
            {
                Serial.printf("[CAM-DBG] READY -> camReady=true, heartbeat=%lu\n", now);
            }
            return;
        }

        if (strcmp(line, "NO_WIFI") == 0)
        {
            g_camReady = false;
            if (CAM_DEBUG_VERBOSE)
            {
                Serial.printf("[CAM-DBG] NO_WIFI -> camReady=false, heartbeat=%lu\n", now);
            }
            return;
        }

        if (strncmp(line, "DET,", 4) == 0)
        {
            const char *payload = line + 4;
            const char *split = strrchr(payload, ',');
            if (!split || split == payload)
            {
                return;
            }

            char label[sizeof(g_lastAiLabel)] = {0};
            size_t labelLen = static_cast<size_t>(split - payload);
            if (labelLen >= sizeof(label))
            {
                labelLen = sizeof(label) - 1;
            }
            memcpy(label, payload, labelLen);
            label[labelLen] = '\0';
            trimLabelInPlace(label);

            float conf = static_cast<float>(atof(split + 1));
            g_camReady = true;
            clearAiOfflineFlag();
            setAiState(true, label, conf);
            setAiError(false, now);
            g_lastAiSuccessMs = now;
            Serial.printf("[CAM] DET label=%s conf=%.1f%%\n", label, conf * 100.0f);
            return;
        }

        if (strcmp(line, "NONE") == 0)
        {
            g_camReady = true;
            clearAiOfflineFlag();
            setAiState(false, "none", 0.0f);
            setAiError(false, now);
            g_lastAiSuccessMs = now;
            Serial.println("[CAM] NONE");
            return;
        }

        if (strcmp(line, "AI_OFFLINE") == 0)
        {
            g_camReady = true;
            g_aiOfflineReported = true;
            setAiState(false, "none", 0.0f);
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE);
            Serial.println("[CAM] AI OFFLINE");
        }
    }
}

void expireAiStateIfStale(uint32_t now)
{
    checkDeviceAndServiceStatus(now);
}

void pollCameraUart()
{
    static uint32_t s_totalBytesReceived = 0;
    static uint32_t s_lastByteReportMs = 0;

    while (CameraUart.available())
    {
        const char c = static_cast<char>(CameraUart.read());
        s_totalBytesReceived++;

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            g_camLineBuf[g_camLinePos] = '\0';
            parseCameraLine(g_camLineBuf);
            g_camLinePos = 0;
            continue;
        }

        if (g_camLinePos < sizeof(g_camLineBuf) - 1)
        {
            g_camLineBuf[g_camLinePos++] = c;
        }
        else
        {
            // 缓冲区溢出，丢弃当前行
            if (CAM_DEBUG_VERBOSE)
            {
                Serial.println("[CAM-DBG] WARN: line buffer overflow, drop current line");
            }
            g_camLinePos = 0;
        }
    }

    // 每5秒报告一次接收统计
    uint32_t now = millis();
    if (CAM_DEBUG_VERBOSE && (now - s_lastByteReportMs) >= 5000)
    {
        s_lastByteReportMs = now;
        Serial.printf("[CAM-DBG] UART STATS: total_rx=%lu bytes\n", s_totalBytesReceived);
    }
}

void updateServoByAiResult()
{
    static const uint32_t SERVO_HOLD_MS = 2000;
    static const uint32_t REARM_SAME_VIEW_MS = 3500;
    static const int SERVO_CONFIRM_FRAMES = 2;
    static int activeServo = 0;
    static uint32_t triggerTimeMs = 0;
    static bool waitForRearm = false;
    static char labelAtLastSort[sizeof(g_lastAiLabel)] = "";
    static uint32_t sortCompletedMs = 0;
    static int confirmCount = 0;
    static char confirmLabel[sizeof(g_lastAiLabel)] = "";

    uint32_t now = millis();

    if (activeServo != 0)
    {
        if (now - triggerTimeMs >= SERVO_HOLD_MS)
        {
            if (activeServo == 1)
            {
                setServoAngle(0);
                Serial.printf("[SERVO] HOME ch=1 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN,
                              static_cast<unsigned long>(SERVO_HOLD_MS));
            }
            else if (activeServo == 2)
            {
                setServoAngle2(0);
                Serial.printf("[SERVO] HOME ch=2 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_2,
                              static_cast<unsigned long>(SERVO_HOLD_MS));
            }
            else if (activeServo == 3)
            {
                setServoAngle3(0);
                Serial.printf("[SERVO] HOME ch=3 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_3,
                              static_cast<unsigned long>(SERVO_HOLD_MS));
            }
            activeServo = 0;
            waitForRearm = true;
            strncpy(labelAtLastSort, g_lastAiLabel, sizeof(labelAtLastSort) - 1);
            labelAtLastSort[sizeof(labelAtLastSort) - 1] = '\0';
            sortCompletedMs = now;
        }
        return;
    }

    if (waitForRearm)
    {
        if (!g_lastAiDetected)
        {
            waitForRearm = false;
        }
        else if (!labelEqualsCi(g_lastAiLabel, labelAtLastSort))
        {
            waitForRearm = false;
        }
        else if ((now - sortCompletedMs) >= REARM_SAME_VIEW_MS)
        {
            waitForRearm = false;
        }
        else
        {
            return;
        }
    }

    if (!g_lastAiDetected || (g_aiConfThreshold > 0.0f && g_lastAiConf < g_aiConfThreshold))
    {
        confirmCount = 0;
        confirmLabel[0] = '\0';
        return;
    }

    if (!labelEqualsCi(g_lastAiLabel, confirmLabel))
    {
        confirmCount = 1;
        strncpy(confirmLabel, g_lastAiLabel, sizeof(confirmLabel) - 1);
        confirmLabel[sizeof(confirmLabel) - 1] = '\0';
        Serial.printf("[SERVO] CONFIRM reset label=%s (need %d frames)\n", confirmLabel, SERVO_CONFIRM_FRAMES);
        return;
    }
    confirmCount++;
    if (confirmCount < SERVO_CONFIRM_FRAMES)
    {
        Serial.printf("[SERVO] CONFIRM accumulate label=%s count=%d/%d\n",
                      confirmLabel, confirmCount, SERVO_CONFIRM_FRAMES);
        return;
    }

    confirmCount = 0;
    confirmLabel[0] = '\0';

    const bool isBattery = labelEqualsCi(g_lastAiLabel, "Battery");
    const bool isMobile = labelEqualsCi(g_lastAiLabel, "MobilePhone");
    const bool isAccessory = labelEqualsCi(g_lastAiLabel, "Charger") ||
                             labelEqualsCi(g_lastAiLabel, "Earphone");

    if (isBattery)
    {
        setServoAngle(180);
        setServoAngle2(0);
        setServoAngle3(0);
        activeServo = 1;
        triggerTimeMs = now;
        Serial.printf("[SERVO] TRIGGER ch=1 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                      SERVO_PIN, g_lastAiLabel, g_lastAiConf * 100.0f,
                      static_cast<unsigned long>(SERVO_HOLD_MS));
    }
    else if (isMobile)
    {
        setServoAngle(0);
        setServoAngle2(180);
        setServoAngle3(0);
        activeServo = 2;
        triggerTimeMs = now;
        Serial.printf("[SERVO] TRIGGER ch=2 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                      SERVO_PIN_2, g_lastAiLabel, g_lastAiConf * 100.0f,
                      static_cast<unsigned long>(SERVO_HOLD_MS));
    }
    else if (isAccessory)
    {
        setServoAngle(0);
        setServoAngle2(0);
        setServoAngle3(180);
        activeServo = 3;
        triggerTimeMs = now;
        Serial.printf("[SERVO] TRIGGER ch=3 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n",
                      SERVO_PIN_3, g_lastAiLabel, g_lastAiConf * 100.0f,
                      static_cast<unsigned long>(SERVO_HOLD_MS));
    }
}
