#include "cameraAi.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

#include "appState.h"
#include "oledInit.h"
#include "servoControl.h"

namespace
{
    bool isPrintableProtocolChar(char c)
    {
        const unsigned char uc = static_cast<unsigned char>(c);
        return uc >= 0x20 && uc <= 0x7E;
    }

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

    bool isProtocolLabelSafe(const char *label)
    {
        // DET 帧使用逗号分隔，标签中若含逗号或控制字符会破坏协议解析。
        if (label == nullptr || label[0] == '\0')
        {
            return false;
        }

        for (const char *p = label; *p != '\0'; ++p)
        {
            if (*p == ',' || !isPrintableProtocolChar(*p))
            {
                return false;
            }
        }

        return true;
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
        // 全局 AI 状态同时供舵机控制和 OLED 页面读取，因此在这里集中更新。
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

    bool isCameraHeartbeatFresh(uint32_t now)
    {
        return g_lastCamHeartbeatMs != 0 && (now - g_lastCamHeartbeatMs) <= CAM_HEARTBEAT_TIMEOUT_MS;
    }

    bool isAiResultFresh(uint32_t now)
    {
        return g_lastAiUpdateMs != 0 && (now - g_lastAiUpdateMs) <= AI_OFFLINE_TIMEOUT_MS;
    }

    void clearActionableAiResult(uint32_t now)
    {
        if (!g_lastAiDetected && strcmp(g_lastAiLabel, "none") == 0 && g_lastAiConf == 0.0f)
        {
            return;
        }

        g_lastAiDetected = false;
        g_lastAiConf = 0.0f;
        g_lastAiUpdateMs = now;
        strncpy(g_lastAiLabel, "none", sizeof(g_lastAiLabel) - 1);
        g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';
        setAiResult(false, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs);
    }

    bool isAiResultActionable(uint32_t now)
    {
        return g_lastAiDetected &&
               isAiResultFresh(now) &&
               g_camReady &&
               !g_aiOfflineReported &&
               isCameraHeartbeatFresh(now);
    }

    void checkDeviceAndServiceStatus(uint32_t now)
    {
        // 重新获取当前时间，避免与 parseCameraLine 中更新的时间戳不一致
        // （loop 开始时的 now 可能比 pollCameraUart 中更新的 g_lastCamHeartbeatMs 旧）
        now = millis();

        const bool heartbeatMissing = !isCameraHeartbeatFresh(now);

        // 摄像头本体无心跳时优先显示 CAM 离线，而不是把问题误判为模型服务异常。
        if (heartbeatMissing || !g_camReady)
        {
            clearActionableAiResult(now);
            setAiError(true, now, AI_ERR_CAM_OFFLINE);
            return;
        }

        // 摄像头在线但后端上报 AI_OFFLINE 时，说明问题位于推理服务或网络到服务器链路。
        if (g_aiOfflineReported)
        {
            clearActionableAiResult(now);
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE);
            return;
        }

        if (g_lastAiDetected && !isAiResultFresh(now))
        {
            clearActionableAiResult(now);
        }

        setAiError(false, now);
    }

    void parseCameraLine(const char *line)
    {
        // 串口协议以一行一帧传输，空行和无效指针直接忽略，避免污染上一帧状态。
        if (line == nullptr || line[0] == '\0')
        {
            return;
        }

        while (*line == ' ' || *line == '\t')
        {
            ++line;
        }

        uint32_t now = millis();

        // NO_WIFI 表示 ESP32-CAM 仍在线，但它自身未连上局域网。
        if (strcmp(line, "NO_WIFI") == 0)
        {
            g_lastCamHeartbeatMs = now;
            g_camReady = false;
            return;
        }

        // DET,<label>,<conf> 是唯一会触发分拣候选的协议帧，格式必须严格校验。
        if (strncmp(line, "DET,", 4) == 0)
        {
            const char *payload = line + 4;
            const char *split = strchr(payload, ',');
            // 标签和置信度只能各出现一次，避免半包或脏数据误触发舵机。
            if (!split || split == payload || strchr(split + 1, ',') != nullptr)
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
            // 标签清洗后仍不安全时丢弃本帧，保留上一帧状态等待下一次有效输入。
            if (!isProtocolLabelSafe(label))
            {
                return;
            }

            float conf = static_cast<float>(atof(split + 1));
            g_lastCamHeartbeatMs = now;
            g_camReady = true;
            clearAiOfflineFlag();
            setAiState(true, label, conf);
            setAiError(false, now);
            g_lastAiSuccessMs = now;
            return;
        }

        // NONE 是一次有效推理结果：视野内无目标，不等同于 AI 服务离线。
        if (strcmp(line, "NONE") == 0)
        {
            g_lastCamHeartbeatMs = now;
            g_camReady = true;
            clearAiOfflineFlag();
            setAiState(false, "none", 0.0f);
            setAiError(false, now);
            g_lastAiSuccessMs = now;
            return;
        }

        // AI_OFFLINE 由 ESP32-CAM 在后端请求失败时发出，用于区分相机在线但服务不可用。
        if (strcmp(line, "AI_OFFLINE") == 0)
        {
            g_lastCamHeartbeatMs = now;
            g_camReady = true;
            g_aiOfflineReported = true;
            g_lastAiSuccessMs = now;
            setAiState(false, "none", 0.0f);
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE);
            return;
        }

        // READY 是相机心跳，仅刷新设备在线状态，不改变最近一次识别结果。
        if (strcmp(line, "READY") == 0)
        {
            g_lastCamHeartbeatMs = now;
            g_camReady = true;
            return;
        }
    }
}

void expireAiStateIfStale(uint32_t now)
{
    checkDeviceAndServiceStatus(now);
}

void pollCameraUart()
{
    static bool s_discardCurrentLine = false;

    while (CameraUart.available())
    {
        const char c = static_cast<char>(CameraUart.read());

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            // 一行结束后再解析，保证协议帧完整；溢出行会被整行丢弃。
            if (!s_discardCurrentLine)
            {
                g_camLineBuf[g_camLinePos] = '\0';
                parseCameraLine(g_camLineBuf);
            }
            g_camLinePos = 0;
            s_discardCurrentLine = false;
            continue;
        }

        if (s_discardCurrentLine)
        {
            continue;
        }

        // 非 ASCII 可打印字符通常来自串口噪声，直接丢弃，避免污染协议缓存。
        if (!isPrintableProtocolChar(c))
        {
            continue;
        }

        // 缓冲区预留一个 '\0'，超过长度说明当前帧异常，丢弃到下一行重新同步。
        if (g_camLinePos < sizeof(g_camLineBuf) - 1)
        {
            g_camLineBuf[g_camLinePos++] = c;
        }
        else
        {
            // 缓冲区溢出，丢弃当前行
            g_camLinePos = 0;
            s_discardCurrentLine = true;
        }
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
        // 舵机动作保持固定时间后回零，避免持续顶住机械结构。
        if (now - triggerTimeMs >= SERVO_HOLD_MS)
        {
            if (activeServo == 1)
            {
                setServoAngle(0);
            }
            else if (activeServo == 2)
            {
                setServoAngle2(0);
            }
            else if (activeServo == 3)
            {
                setServoAngle3(0);
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
        // 防重复触发：同一目标停留在画面内时，等待目标消失、标签变化或超时后再允许下一次分拣。
        if (!isAiResultActionable(now))
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

    // 未检测到有效新结果、结果过期或低于阈值时清空连续确认计数，避免上一目标残留触发。
    if (!isAiResultActionable(now) || (g_aiConfThreshold > 0.0f && g_lastAiConf < g_aiConfThreshold))
    {
        confirmCount = 0;
        confirmLabel[0] = '\0';
        return;
    }

    // 连续两帧同类才触发舵机，用较小延迟换取抗抖动能力。
    if (!labelEqualsCi(g_lastAiLabel, confirmLabel))
    {
        confirmCount = 1;
        strncpy(confirmLabel, g_lastAiLabel, sizeof(confirmLabel) - 1);
        confirmLabel[sizeof(confirmLabel) - 1] = '\0';
        return;
    }
    confirmCount++;
    if (confirmCount < SERVO_CONFIRM_FRAMES)
    {
        return;
    }

    confirmCount = 0;
    confirmLabel[0] = '\0';

    const bool isBattery = labelEqualsCi(g_lastAiLabel, "Battery");
    const bool isMobile = labelEqualsCi(g_lastAiLabel, "MobilePhone");
    const bool isAccessory = labelEqualsCi(g_lastAiLabel, "Charger") ||
                             labelEqualsCi(g_lastAiLabel, "Earphone");

    // 标签到仓位的映射与前端统计分类保持一致：电池、手机、配件分别进入三路仓体。
    if (isBattery)
    {
        setServoAngle(180);
        setServoAngle2(0);
        setServoAngle3(0);
        activeServo = 1;
        triggerTimeMs = now;
    }
    else if (isMobile)
    {
        setServoAngle(0);
        setServoAngle2(180);
        setServoAngle3(0);
        activeServo = 2;
        triggerTimeMs = now;
    }
    else if (isAccessory)
    {
        setServoAngle(0);
        setServoAngle2(0);
        setServoAngle3(180);
        activeServo = 3;
        triggerTimeMs = now;
    }
}
