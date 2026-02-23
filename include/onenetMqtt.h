#ifndef _ONENET_MQTT_H_
#define _ONENET_MQTT_H_

#include <Arduino.h>
#include "onenetToken.h"

// OneNET MQTT 连接配置。
// 说明：
// - host/port: MQTT broker 地址与端口。
// - productId/deviceName: 用于 MQTT 登录和 Topic 组装。
// - base64Key: 用于生成 token 的 Base64 密钥。
// - tokenExpireAt: token 过期时间（Unix 秒级时间戳）。
// - signMethod: token 签名算法。
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

// 初始化 OneNET MQTT 客户端配置（仅设置，不阻塞等待连接）。
void oneNetMqttBegin(const OneNetMqttConfig &config);

// MQTT 主循环：建议在 Arduino loop() 中持续调用。
// 作用：维护连接、执行自动重连、处理订阅消息收发。
void oneNetMqttLoop();

// 查询当前 MQTT 连接状态。
bool oneNetMqttConnected();

// 按 OneJSON 物模型属性上报格式发布数据。
// 对应你当前模型中的两个属性：
// - isFull(bool)
// - weight(int32)
bool oneNetMqttUploadProperties(bool isFull, int32_t weight);

#endif