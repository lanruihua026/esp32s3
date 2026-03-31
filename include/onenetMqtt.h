#ifndef _ONENET_MQTT_H_
#define _ONENET_MQTT_H_

#include "onenetToken.h"
#include <Arduino.h>

/**
 * @brief OneNET MQTT 连接参数
 *
 * 字段说明：
 * - host / port: MQTT 服务器地址与端口。
 * - productId / deviceName: 设备身份，用于登录和 Topic 拼接。
 * - base64Key: 设备密钥（Base64 编码）。
 * - tokenExpireAt: token 过期时间（Unix 秒级时间戳）。
 * - signMethod: token 签名算法。
 */
struct OneNetMqttConfig
{
    const char *host;
    uint16_t port;
    const char *productId;
    const char *deviceName;
    const char *base64Key;
    uint32_t tokenExpireAt;
    OneNetSignMethod signMethod;
};

/**
 * @brief 初始化 OneNET MQTT 模块
 *
 * 注意：
 * - 此函数只做参数与客户端初始化，不会阻塞等待连上 MQTT。
 * - 真正的连接与重连在 oneNetMqttLoop() 中执行。
 */
void oneNetMqttBegin(const OneNetMqttConfig &config);

/**
 * @brief MQTT 主循环维护函数
 *
 * 建议在 Arduino loop() 中持续调用。
 * 功能包括：
 * 1. 自动重连；
 * 2. 维持心跳；
 * 3. 处理订阅消息收发。
 */
void oneNetMqttLoop();

/**
 * @brief 查询当前 MQTT 连接状态
 */
bool oneNetMqttConnected();

/**
 * @brief 单个仓格的物模型数据
 *
 * 字段说明：
 * - weight:   当前重量（g），对应物模型 *_weight，范围 0~5000
 * - percent:  满溢百分比（%），对应物模型 *_percent，范围 0.00~100.00
 * - nearFull: 是否即将满载（重量 >= 满载阈值的 90%），对应物模型 *_near_full
 * - full:     是否满溢（重量 >= 满载阈值），对应物模型 *_full
 */
struct BoxBinData
{
    int32_t weight; // 当前重量 (g)
    float percent;  // 满溢百分比 (%)
    bool nearFull;  // 是否即将满载（>= 90% 阈值）
    bool full;      // 是否满溢
};

/**
 * @brief 以 OneJSON 属性上报格式发布三仓数据 + 满载阈值 + AI 置信度阈值（共 14 个属性）
 * @param phone             手机仓数据（含 nearFull / full 两个布尔状态）
 * @param mouse             鼠标仓数据（含 nearFull / full 两个布尔状态）
 * @param battery           电池仓数据（含 nearFull / full 两个布尔状态）
 * @param overflowThresholdG 当前满载阈值（克），同步上报使云平台显示实际值而非 undefined
 * @param aiConfThreshold   当前 AI 识别置信度阈值（0~1），同步上报供云端与网页展示
 */
bool oneNetMqttUploadProperties(
    const BoxBinData &phone,
    const BoxBinData &mouse,
    const BoxBinData &battery,
    int32_t overflowThresholdG,
    float aiConfThreshold);

/**
 * @brief 注册平台属性下发（property/set）回调
 *
 * 当 OneNET 平台通过 MQTT 下发属性设置消息时，回调函数将被调用。
 * @param cb 回调函数指针，参数为消息载荷（非 null 结尾）和长度；传 nullptr 取消注册。
 *
 * 使用示例（main.cpp 中）：
 *   oneNetSetPropertySetCallback([](const char* payload, unsigned int len) {
 *       // 解析 OneJSON payload，更新本地阈值
 *   });
 */
void oneNetSetPropertySetCallback(void (*cb)(const char *payload, unsigned int len));

/**
 * @brief 主动断开 MQTT 连接
 *
 * 在 WiFi 掉线时由主循环调用，防止 TCP 半开状态导致后续属性上报静默失败。
 * 断开后 oneNetMqttLoop() 会在 WiFi 恢复时自动重连，无需额外处理。
 */
void oneNetMqttDisconnect();

#endif
