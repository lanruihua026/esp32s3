#include "cameraAi.h" // 摄像头 AI 模块头文件

#include <cctype>  // 字符处理函数
#include <cstdlib> // 标准库工具函数
#include <cstring> // C 字符串函数

#include "appState.h"     // 全局运行状态
#include "oledInit.h"     // OLED 显示相关接口
#include "servoControl.h" // 舵机控制相关接口

namespace // 匿名命名空间，限制内部工具函数作用域
{         // 匿名命名空间开始
    // 调试输出控制：设为 true 可打印详细的 UART 接收和状态检测信息
    static constexpr bool CAM_DEBUG_VERBOSE = true; // 是否输出详细调试日志
    // 周期性状态报告间隔（毫秒）
    static constexpr uint32_t CAM_STATUS_REPORT_INTERVAL_MS = 5000; // 状态报告周期
    static uint32_t s_lastStatusReportMs = 0;                       // 上次状态报告时间

    void trimLabelInPlace(char *s)        // 去掉字符串尾部空白
    {                                     // 函数体开始
        if (s == nullptr || s[0] == '\0') // 空指针或空字符串直接返回
        {                                 // if 开始
            return;                       // 没有可处理内容
        } // 结束空串判断
        size_t n = strlen(s);                                                      // 计算字符串长度
        while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r')) // 从尾部去掉空白字符
        {                                                                          // while 开始
            s[--n] = '\0';                                                         // 把末尾空白替换成结束符
        } // 结束尾部空白清理
    } // 结束 trimLabelInPlace

    bool labelEqualsCi(const char *a, const char *b) // 忽略大小写比较两个标签
    {                                                // 函数体开始
        if (a == nullptr || b == nullptr)            // 任一指针为空就认为不相等
        {                                            // if 开始
            return false;                            // 无效输入直接返回 false
        } // 结束空指针判断
        while (*a != '\0' && *b != '\0')                               // 两个字符串都未结束时继续比较
        {                                                              // while 开始
            const unsigned char ca = static_cast<unsigned char>(*a++); // 读取 a 当前字符并前移
            const unsigned char cb = static_cast<unsigned char>(*b++); // 读取 b 当前字符并前移
            if (std::tolower(ca) != std::tolower(cb))                  // 比较小写形式是否一致
            {                                                          // if 开始
                return false;                                          // 只要有一处不同就不相等
            } // 结束字符比较
        } // 结束逐字符比较
        return *a == *b; // 两个字符串必须同时结束才算相等
    } // 结束 labelEqualsCi

    void setAiState(bool detected, const char *label, float conf) // 更新 AI 检测状态
    {                                                             // 函数体开始
        g_lastAiDetected = detected;                              // 记录是否检测到目标
        g_lastAiConf = conf;                                      // 记录当前置信度
        g_lastAiUpdateMs = millis();                              // 记录状态更新时间

        strncpy(g_lastAiLabel, label, sizeof(g_lastAiLabel) - 1); // 复制标签到全局缓冲区
        g_lastAiLabel[sizeof(g_lastAiLabel) - 1] = '\0';          // 确保字符串结尾安全

        setAiResult(detected, g_lastAiLabel, g_lastAiConf, g_lastAiUpdateMs); // 同步到外部 AI 结果状态
    } // 结束 setAiState

    void clearAiOfflineFlag()        // 清除 AI 离线标记
    {                                // 函数体开始
        g_aiOfflineReported = false; // 允许重新进入正常状态
    } // 结束 clearAiOfflineFlag

    void checkDeviceAndServiceStatus(uint32_t now) // 检查摄像头和 AI 服务状态
    {                                              // 函数体开始
        // 重新获取当前时间，避免与 parseCameraLine 中更新的时间戳不一致
        // （loop 开始时的 now 可能比 pollCameraUart 中更新的 g_lastCamHeartbeatMs 旧）
        now = millis(); // 重新读取当前时间

        const bool heartbeatMissing =                                                               // 判断摄像头心跳是否超时
            (g_lastCamHeartbeatMs == 0 || (now - g_lastCamHeartbeatMs) > CAM_HEARTBEAT_TIMEOUT_MS); // 超时条件

        // 周期性状态报告
        if (CAM_DEBUG_VERBOSE && (now - s_lastStatusReportMs) >= CAM_STATUS_REPORT_INTERVAL_MS) // 到达报告间隔
        {                                                                                       // if 开始
            s_lastStatusReportMs = now;                                                         // 更新报告时间
            Serial.printf("[CAM-DBG] === STATUS REPORT ===\n");                                 // 打印状态标题
            Serial.printf("[CAM-DBG] now=%lu, lastHeartbeat=%lu, diff=%lu ms\n",                // 打印当前时间和心跳差值
                          now,                                                                  // 当前时间
                          g_lastCamHeartbeatMs,                                                 // 最近一次心跳时间
                          g_lastCamHeartbeatMs > 0 ? (now - g_lastCamHeartbeatMs) : 0);         // 心跳间隔
            Serial.printf("[CAM-DBG] lastAiSuccess=%lu, diff=%lu ms\n",                         // 打印最近一次识别成功时间
                          g_lastAiSuccessMs,                                                    // 最近一次识别成功时间
                          g_lastAiSuccessMs > 0 ? (now - g_lastAiSuccessMs) : 0);               // AI 成功间隔
            Serial.printf("[CAM-DBG] camReady=%d, aiOfflineReported=%d\n",                      // 打印摄像头和离线标记
                          g_camReady ? 1 : 0, g_aiOfflineReported ? 1 : 0);                     // 布尔状态转成数字
            Serial.printf("[CAM-DBG] heartbeatMissing=%d (timeout=%lu ms)\n",                   // 打印心跳是否超时
                          heartbeatMissing ? 1 : 0, CAM_HEARTBEAT_TIMEOUT_MS);                  // 超时时间
        } // 结束状态报告

        if (heartbeatMissing || !g_camReady)                                                  // 摄像头无心跳或未就绪
        {                                                                                     // if 开始
            if (CAM_DEBUG_VERBOSE)                                                            // 需要打印离线原因
            {                                                                                 // if 开始
                Serial.printf("[CAM-DBG] -> CAM_OFFLINE: heartbeatMissing=%d, camReady=%d\n", // 打印摄像头离线原因
                              heartbeatMissing ? 1 : 0, g_camReady ? 1 : 0);                  // 输出状态值
            } // 结束调试输出
            setAiError(true, now, AI_ERR_CAM_OFFLINE); // 标记摄像头离线
            return;                                    // 直接结束
        } // 结束摄像头离线判断

        const bool aiTimedOut =                                                                 // 判断 AI 是否长时间无结果
            (g_lastAiSuccessMs == 0 || (now - g_lastAiSuccessMs) > AI_OFFLINE_TIMEOUT_MS);      // 超时条件
        if (g_aiOfflineReported || aiTimedOut)                                                  // 已主动报离线或已超时
        {                                                                                       // if 开始
            if (CAM_DEBUG_VERBOSE)                                                              // 需要打印 AI 离线原因
            {                                                                                   // if 开始
                Serial.printf("[CAM-DBG] -> AI_OFFLINE: aiOfflineReported=%d, aiTimedOut=%d\n", // 打印 AI 离线原因
                              g_aiOfflineReported ? 1 : 0, aiTimedOut ? 1 : 0);                 // 输出状态值
            } // 结束调试输出
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE); // 标记服务离线
            return;                                        // 直接结束
        } // 结束 AI 离线判断

        setAiError(false, now); // 恢复正常状态
    } // 结束状态检查函数

    void parseCameraLine(const char *line)      // 解析摄像头串口的一行数据
    {                                           // 函数体开始
        if (line == nullptr || line[0] == '\0') // 空指针或空字符串直接返回
        {                                       // if 开始
            return;                             // 没有可解析内容
        } // 结束空输入判断

        while (*line == ' ' || *line == '\t') // 去掉行首空格和制表符
        {                                     // while 开始
            ++line;                           // 指针后移
        } // 结束行首空白处理

        // 调试：打印收到的每行数据
        if (CAM_DEBUG_VERBOSE)                                                         // 开启调试输出时打印原始行
        {                                                                              // if 开始
            Serial.printf("[CAM-DBG] RX LINE: \"%s\" (len=%d)\n", line, strlen(line)); // 输出收到的行内容
        } // 结束调试打印

        uint32_t now = millis();    // 获取当前时间
        g_lastCamHeartbeatMs = now; // 更新摄像头心跳时间

        if (strcmp(line, "READY") == 0)                                                  // 摄像头上报 READY
        {                                                                                // if 开始
            g_camReady = true;                                                           // 标记摄像头已就绪
            if (CAM_DEBUG_VERBOSE)                                                       // 需要打印调试信息
            {                                                                            // if 开始
                Serial.printf("[CAM-DBG] READY -> camReady=true, heartbeat=%lu\n", now); // 打印就绪状态
            } // 结束调试输出
            return; // 结束 READY 处理
        } // 结束 READY 判断

        if (strcmp(line, "NO_WIFI") == 0)                                                   // 摄像头上报无 WiFi
        {                                                                                   // if 开始
            g_camReady = false;                                                             // 标记摄像头未就绪
            if (CAM_DEBUG_VERBOSE)                                                          // 需要打印调试信息
            {                                                                               // if 开始
                Serial.printf("[CAM-DBG] NO_WIFI -> camReady=false, heartbeat=%lu\n", now); // 打印无 WiFi 状态
            } // 结束调试输出
            return; // 结束 NO_WIFI 处理
        } // 结束 NO_WIFI 判断

        if (strncmp(line, "DET,", 4) == 0)             // 摄像头上报识别结果
        {                                              // if 开始
            const char *payload = line + 4;            // 跳过 DET, 前缀
            const char *split = strrchr(payload, ','); // 找到最后一个逗号，分隔标签和置信度
            if (!split || split == payload)            // 格式不合法
            {                                          // if 开始
                return;                                // 直接丢弃
            } // 结束格式判断

            char label[sizeof(g_lastAiLabel)] = {0};                // 临时标签缓冲区
            size_t labelLen = static_cast<size_t>(split - payload); // 计算标签长度
            if (labelLen >= sizeof(label))                          // 防止标签过长
            {                                                       // if 开始
                labelLen = sizeof(label) - 1;                       // 保留结尾终止符空间
            } // 结束长度限制
            memcpy(label, payload, labelLen); // 复制标签内容
            label[labelLen] = '\0';           // 手动补字符串结束符
            trimLabelInPlace(label);          // 去掉标签尾部空白

            float conf = static_cast<float>(atof(split + 1));                        // 解析置信度值
            g_camReady = true;                                                       // 摄像头仍然视为就绪
            clearAiOfflineFlag();                                                    // 清除离线标记
            setAiState(true, label, conf);                                           // 更新 AI 检测状态
            setAiError(false, now);                                                  // 清除错误状态
            g_lastAiSuccessMs = now;                                                 // 记录最近一次成功识别时间
            Serial.printf("[CAM] DET label=%s conf=%.1f%%\n", label, conf * 100.0f); // 打印识别结果
            return;                                                                  // 结束 DET 处理
        } // 结束 DET 判断

        if (strcmp(line, "NONE") == 0)       // 摄像头上报未识别到目标
        {                                    // if 开始
            g_camReady = true;               // 摄像头保持就绪
            clearAiOfflineFlag();            // 清除离线标记
            setAiState(false, "none", 0.0f); // 更新为未检测到目标
            setAiError(false, now);          // 清除错误状态
            g_lastAiSuccessMs = now;         // 记录本次成功通信时间
            Serial.println("[CAM] NONE");    // 打印未识别到目标
            return;                          // 结束 NONE 处理
        } // 结束 NONE 判断

        if (strcmp(line, "AI_OFFLINE") == 0)               // 摄像头上报 AI 服务离线
        {                                                  // if 开始
            g_camReady = true;                             // 摄像头本体仍可通信
            g_aiOfflineReported = true;                    // 记录 AI 离线标记
            setAiState(false, "none", 0.0f);               // 清空当前识别结果
            setAiError(true, now, AI_ERR_SERVICE_OFFLINE); // 标记服务离线
            Serial.println("[CAM] AI OFFLINE");            // 打印 AI 离线提示
        } // 结束 AI_OFFLINE 判断
    } // 结束 parseCameraLine
} // 结束匿名命名空间

void expireAiStateIfStale(uint32_t now) // 检查 AI 状态是否过期
{                                       // 函数体开始
    checkDeviceAndServiceStatus(now);   // 复用状态检查逻辑
} // 结束 expireAiStateIfStale

void pollCameraUart()                         // 轮询摄像头串口
{                                             // 函数体开始
    static uint32_t s_totalBytesReceived = 0; // 累计接收字节数
    static uint32_t s_lastByteReportMs = 0;   // 上次统计输出时间

    while (CameraUart.available())                           // 只要串口还有数据就继续读
    {                                                        // while 开始
        const char c = static_cast<char>(CameraUart.read()); // 读取一个字节
        s_totalBytesReceived++;                              // 累加接收字节数

        if (c == '\r') // 丢弃回车符
        {              // if 开始
            continue;  // 继续读取下一个字节
        } // 结束回车处理

        if (c == '\n')                         // 遇到换行，说明一行数据结束
        {                                      // if 开始
            g_camLineBuf[g_camLinePos] = '\0'; // 给当前缓冲区补结束符
            parseCameraLine(g_camLineBuf);     // 解析完整的一行
            g_camLinePos = 0;                  // 重置写入位置
            continue;                          // 开始接收下一行
        } // 结束换行处理

        if (g_camLinePos < sizeof(g_camLineBuf) - 1) // 缓冲区还有空间
        {                                            // if 开始
            g_camLineBuf[g_camLinePos++] = c;        // 写入字符并推进位置
        } // 结束正常写入
        else // 缓冲区已满
        {    // else 开始
            // 缓冲区溢出，丢弃当前行
            if (CAM_DEBUG_VERBOSE)                                                         // 需要输出溢出调试信息
            {                                                                              // if 开始
                Serial.println("[CAM-DBG] WARN: line buffer overflow, drop current line"); // 打印溢出警告
            } // 结束调试输出
            g_camLinePos = 0; // 丢弃当前行并重新开始
        } // 结束溢出处理
    } // 结束串口读取循环

    // 每5秒报告一次接收统计
    uint32_t now = millis();                                                               // 获取当前时间
    if (CAM_DEBUG_VERBOSE && (now - s_lastByteReportMs) >= 5000)                           // 到达统计输出周期
    {                                                                                      // if 开始
        s_lastByteReportMs = now;                                                          // 更新统计时间
        Serial.printf("[CAM-DBG] UART STATS: total_rx=%lu bytes\n", s_totalBytesReceived); // 打印累计接收字节数
    } // 结束统计输出
} // 结束摄像头串口轮询

void updateServoByAiResult()                                 // 根据 AI 结果更新舵机
{                                                            // 函数体开始
    static const uint32_t SERVO_HOLD_MS = 2000;              // 舵机保持动作时间
    static const uint32_t REARM_SAME_VIEW_MS = 3500;         // 同一视角重新触发等待时间
    static const int SERVO_CONFIRM_FRAMES = 2;               // 连续确认帧数要求
    static int activeServo = 0;                              // 当前正在动作的舵机编号
    static uint32_t triggerTimeMs = 0;                       // 当前舵机触发时间
    static bool waitForRearm = false;                        // 是否等待重新布防
    static char labelAtLastSort[sizeof(g_lastAiLabel)] = ""; // 上次分拣标签
    static uint32_t sortCompletedMs = 0;                     // 上次分拣完成时间
    static int confirmCount = 0;                             // 连续确认计数
    static char confirmLabel[sizeof(g_lastAiLabel)] = "";    // 当前确认中的标签

    uint32_t now = millis(); // 获取当前时间

    if (activeServo != 0)                                                                             // 当前有舵机正在保持动作
    {                                                                                                 // if 开始
        if (now - triggerTimeMs >= SERVO_HOLD_MS)                                                     // 保持时间到
        {                                                                                             // if 开始
            if (activeServo == 1)                                                                     // 1 号舵机
            {                                                                                         // if 开始
                setServoAngle(0);                                                                     // 复位 1 号舵机
                Serial.printf("[SERVO] HOME ch=1 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN, // 打印 1 号舵机复位日志
                              static_cast<unsigned long>(SERVO_HOLD_MS));                             // 打印保持时间
            } // 结束 1 号舵机处理
            else if (activeServo == 2)                                                                  // 2 号舵机
            {                                                                                           // else-if 开始
                setServoAngle2(0);                                                                      // 复位 2 号舵机
                Serial.printf("[SERVO] HOME ch=2 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_2, // 打印 2 号舵机复位日志
                              static_cast<unsigned long>(SERVO_HOLD_MS));                               // 打印保持时间
            } // 结束 2 号舵机处理
            else if (activeServo == 3)                                                                  // 3 号舵机
            {                                                                                           // else-if 开始
                setServoAngle3(0);                                                                      // 复位 3 号舵机
                Serial.printf("[SERVO] HOME ch=3 GPIO%d angle=0 deg (hold %lu ms done)\n", SERVO_PIN_3, // 打印 3 号舵机复位日志
                              static_cast<unsigned long>(SERVO_HOLD_MS));                               // 打印保持时间
            } // 结束 3 号舵机处理
            activeServo = 0;                                                      // 清除当前活动舵机
            waitForRearm = true;                                                  // 进入重新触发等待期
            strncpy(labelAtLastSort, g_lastAiLabel, sizeof(labelAtLastSort) - 1); // 记录本次分类标签
            labelAtLastSort[sizeof(labelAtLastSort) - 1] = '\0';                  // 确保字符串结尾安全
            sortCompletedMs = now;                                                // 记录分类结束时间
        } // 结束保持时间判断
        return; // 舵机仍在动作中时不继续判断新目标
    } // 结束活动舵机判断

    if (waitForRearm)             // 正在等待同视角重新触发
    {                             // if 开始
        if (!g_lastAiDetected)    // 当前没有检测到目标
        {                         // if 开始
            waitForRearm = false; // 结束等待
        } // 结束未检测到目标处理
        else if (!labelEqualsCi(g_lastAiLabel, labelAtLastSort)) // 标签已经变化
        {                                                        // else-if 开始
            waitForRearm = false;                                // 允许重新触发
        } // 结束标签变化处理
        else if ((now - sortCompletedMs) >= REARM_SAME_VIEW_MS) // 等待时间已到
        {                                                       // else-if 开始
            waitForRearm = false;                               // 允许重新触发
        } // 结束等待时间判断
        else        // 仍需等待
        {           // else 开始
            return; // 暂不触发
        } // 结束等待分支
    } // 结束重新布防等待

    if (!g_lastAiDetected || (g_aiConfThreshold > 0.0f && g_lastAiConf < g_aiConfThreshold)) // 目标无效或置信度不足
    {                                                                                        // if 开始
        confirmCount = 0;                                                                    // 清空连续确认计数
        confirmLabel[0] = '\0';                                                              // 清空确认标签
        return;                                                                              // 不触发舵机
    } // 结束检测条件判断

    if (!labelEqualsCi(g_lastAiLabel, confirmLabel))                                                            // 新标签出现
    {                                                                                                           // if 开始
        confirmCount = 1;                                                                                       // 从第一帧开始确认
        strncpy(confirmLabel, g_lastAiLabel, sizeof(confirmLabel) - 1);                                         // 记录新标签
        confirmLabel[sizeof(confirmLabel) - 1] = '\0';                                                          // 确保字符串结尾安全
        Serial.printf("[SERVO] CONFIRM reset label=%s (need %d frames)\n", confirmLabel, SERVO_CONFIRM_FRAMES); // 打印确认重置日志
        return;                                                                                                 // 还未达到触发条件
    } // 结束新标签判断
    confirmCount++;                                                        // 累加确认帧数
    if (confirmCount < SERVO_CONFIRM_FRAMES)                               // 确认帧数不足
    {                                                                      // if 开始
        Serial.printf("[SERVO] CONFIRM accumulate label=%s count=%d/%d\n", // 打印确认累计日志
                      confirmLabel, confirmCount, SERVO_CONFIRM_FRAMES);   // 输出当前帧数
        return;                                                            // 继续等待
    } // 结束确认帧数判断

    confirmCount = 0;       // 重置确认计数
    confirmLabel[0] = '\0'; // 清空确认标签

    const bool isBattery = labelEqualsCi(g_lastAiLabel, "Battery");     // 判断是否为电池
    const bool isMobile = labelEqualsCi(g_lastAiLabel, "MobilePhone");  // 判断是否为手机
    const bool isAccessory = labelEqualsCi(g_lastAiLabel, "Charger") || // 判断是否为充电器
                             labelEqualsCi(g_lastAiLabel, "Earphone");  // 判断是否为耳机

    if (isBattery)                                                                                      // 电池目标
    {                                                                                                   // if 开始
        setServoAngle(180);                                                                             // 1 号舵机转到分拣位
        setServoAngle2(0);                                                                              // 2 号舵机复位
        setServoAngle3(0);                                                                              // 3 号舵机复位
        activeServo = 1;                                                                                // 记录当前动作舵机
        triggerTimeMs = now;                                                                            // 记录触发时间
        Serial.printf("[SERVO] TRIGGER ch=1 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n", // 打印 1 号舵机触发日志
                      SERVO_PIN, g_lastAiLabel, g_lastAiConf * 100.0f,                                  // 输出引脚、标签和置信度
                      static_cast<unsigned long>(SERVO_HOLD_MS));                                       // 输出保持时间
    } // 结束电池分支
    else if (isMobile)                                                                                  // 手机目标
    {                                                                                                   // else-if 开始
        setServoAngle(0);                                                                               // 1 号舵机复位
        setServoAngle2(180);                                                                            // 2 号舵机转到分拣位
        setServoAngle3(0);                                                                              // 3 号舵机复位
        activeServo = 2;                                                                                // 记录当前动作舵机
        triggerTimeMs = now;                                                                            // 记录触发时间
        Serial.printf("[SERVO] TRIGGER ch=2 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n", // 打印 2 号舵机触发日志
                      SERVO_PIN_2, g_lastAiLabel, g_lastAiConf * 100.0f,                                // 输出引脚、标签和置信度
                      static_cast<unsigned long>(SERVO_HOLD_MS));                                       // 输出保持时间
    } // 结束手机分支
    else if (isAccessory)                                                                               // 配件目标
    {                                                                                                   // else-if 开始
        setServoAngle(0);                                                                               // 1 号舵机复位
        setServoAngle2(0);                                                                              // 2 号舵机复位
        setServoAngle3(180);                                                                            // 3 号舵机转到分拣位
        activeServo = 3;                                                                                // 记录当前动作舵机
        triggerTimeMs = now;                                                                            // 记录触发时间
        Serial.printf("[SERVO] TRIGGER ch=3 GPIO%d angle=180 deg label=%s conf=%.1f%% (hold %lu ms)\n", // 打印 3 号舵机触发日志
                      SERVO_PIN_3, g_lastAiLabel, g_lastAiConf * 100.0f,                                // 输出引脚、标签和置信度
                      static_cast<unsigned long>(SERVO_HOLD_MS));                                       // 输出保持时间
    } // 结束配件分支
} // 结束 updateServoByAiResult