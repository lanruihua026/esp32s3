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
 * @brief 以 OneJSON 属性上报格式发布数据
 * @param isFull 业务属性：是否满载
 * @param weight 业务属性：当前重量
 */
bool oneNetMqttUploadProperties(bool isFull, int32_t weight);

#endif
