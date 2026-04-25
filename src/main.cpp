#include <Arduino.h>       // Arduino 基础功能
#include "appState.h"      // 全局运行状态
#include "binControl.h"    // 仓体控制相关接口
#include "buttonControl.h" // 按键处理相关接口
#include "cameraAi.h"      // 摄像头与 AI 识别相关接口
#include "connectToWiFi.h" // WiFi 连接相关接口
#include "hx711.h"         // HX711 称重传感器相关接口
#include "oledInit.h"      // OLED 显示相关接口
#include "onenetMqtt.h"    // OneNet MQTT 相关接口
#include "systemInit.h"    // 系统初始化相关接口

void setup() // 系统上电初始化入口
{
    Serial.begin(115200);
    g_prefsOk = gPrefs.begin("sysconf", false); // 打开 NVS 配置区，false 表示读写模式

    if (g_prefsOk) // NVS 可用时读取保存的配置
    {
        g_fullWeightG = gPrefs.getInt("full_w", 1000);        // 读取满载重量，默认 1000g
        g_aiConfThreshold = gPrefs.getFloat("ai_conf", 0.0f); // 读取 AI 置信度阈值，默认 0
    }

    if (g_fullWeightG < 100 || g_fullWeightG > 5000) // 检查重量阈值是否越界
    {
        g_fullWeightG = 1000; // 恢复默认值
    }
    if (!(g_aiConfThreshold >= 0.0f && g_aiConfThreshold <= 1.0f)) // 检查模型检测阈值是否在 0~1 之间
    {
        g_aiConfThreshold = 0.0f; // 恢复默认值
        if (g_prefsOk)            // 仅在 NVS 可用时回写
        {
            gPrefs.putFloat("ai_conf", g_aiConfThreshold); // 将修正后的阈值写回配置区
        }
    }
    initBoardIndicators(); // 初始化板载指示灯
    initCameraUart();      // 初始化摄像头串口
    initOledModule();      // 初始化 OLED 显示屏
    startWiFiConnect();    // 开始 WiFi 连接流程
    initHx711Modules();    // 初始化称重传感器模块
    initWiFiWithTimeout(); // 等待 WiFi 连接，带超时控制
    initServoModule();     // 初始化舵机控制模块

    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_RUNNING, "IRQ"); // 标记按键模块正在初始化
    showBootProgress(84, "Buttons");                              // 刷新启动进度条
    initButtonsForOledNavigation();                               // 初始化用于 OLED 菜单导航的按键
    setButton3Callback(silenceOverflowAlarmBuzzer);               // 绑定三号按键的静音回调
    setInitModuleStatus(INIT_MODULE_BUTTON, INIT_OK, "Ready");    // 标记按键模块初始化完成

    initMqttModule();                  // 初始化 MQTT 云端通信模块
    showBootProgress(100, "Starting"); // 启动进度显示到 100%
    delay(200);                        // 短暂等待，给外设一点稳定时间

    initTimers();        // 启动定时器任务
    syncRuntimeHealth(); // 同步当前运行状态
}

void loop() // 主循环，持续执行业务逻辑
{
    const uint32_t now = millis(); // 获取当前运行时间

    pollHx711SerialCommands(gPrefs, g_prefsOk, g_currentWeight1, g_currentWeight2, g_currentWeight3); // 轮询称重传感器串口命令
    pollCameraUart();                                                                                 // 读取摄像头串口数据
    expireAiStateIfStale(now);                                                                        // 检查 AI 状态是否过期
    updateServoByAiResult();                                                                          // 根据 AI 结果更新舵机状态
    pollButtons();                                                                                    // 扫描按键输入

    tryReconnectWiFi(now);           // 发现断线时尝试重连 WiFi
    oneNetMqttLoop();                // 处理 MQTT 收发
    processDeferredPropertyReport(); // 处理延迟上报的属性
    syncRuntimeHealth();             // 同步运行健康状态

    if ((now - g_lastOledUpdateMs) >= OLED_MIN_UPDATE_INTERVAL_MS) // 判断是否到了 OLED 刷新周期
    {
        updateOLEDDisplay();      // 刷新 OLED 显示内容
        g_lastOledUpdateMs = now; // 记录本次刷新时间
    }

    if ((now - g_lastSampleMs) >= WEIGHT_SAMPLE_INTERVAL_MS) // 判断是否到了重量采样周期
    {
        g_lastSampleMs = now;    // 记录本次采样时间
        updateWeightsAndAlarm(); // 更新重量并检查告警
    }
    uploadPropertiesIfNeeded(now); // 按需上报设备属性
}
