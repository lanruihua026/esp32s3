#include "onenetMqtt.h"
#include <PubSubClient.h>
#include <WiFi.h>

namespace
{
    // MQTT 传输底层客户端（当前为明文 TCP）。
    // 如需 TLS，可改为 WiFiClientSecure 并配置证书。
    WiFiClient gWifiClient;
    PubSubClient gMqttClient(gWifiClient);

    // 全局配置（由 oneNetMqttBegin 覆盖）
    OneNetMqttConfig gConfig = {
        "mqtts.heclouds.com",
        1883,
        "",
        "",
        "",
        1893456000,
        OneNetSignMethod::SHA256};

    // 模块初始化状态
    bool gInited = false;

    // 最近一次连接尝试时间（用于重连限流）
    uint32_t gLastConnectAttemptMs = 0;

    // OneNET 相关 Topic 缓存
    String gPropertyPostTopic;
    String gPropertyPostReplyTopic;
    String gPropertySetTopic;

    /**
     * @brief 生成 token 用资源串
     */
    String buildResource()
    {
        return String("products/") + gConfig.productId + "/devices/" + gConfig.deviceName;
    }

    /**
     * @brief 生成消息 id
     *
     * 当前用 millis()，满足一般调试和轻量业务场景。
     */
    String buildMessageId()
    {
        return String(static_cast<uint32_t>(millis()));
    }

    /**
     * @brief MQTT 订阅回调
     *
     * 当前仅打印平台下发和应答消息，便于联调。
     */
    void mqttCallback(char *topic, byte *payload, unsigned int length)
    {
        String msg;
        msg.reserve(length);
        for (unsigned int i = 0; i < length; ++i)
        {
            msg += static_cast<char>(payload[i]);
        }

        // Serial.print("[OneNET] topic: ");
        // Serial.println(topic);
        // Serial.print("[OneNET] payload: ");
        // Serial.println(msg);
    }

    /**
     * @brief 尝试连接 MQTT
     *
     * 连接前置条件：
     * 1. 模块已初始化；
     * 2. WiFi 已连接；
     * 3. MQTT 当前未连接；
     * 4. 距离上次尝试超过 3 秒（限流）。
     */
    void tryConnect()
    {
        if (!gInited || WiFi.status() != WL_CONNECTED || gMqttClient.connected())
        {
            return;
        }

        uint32_t nowMs = millis();
        if (nowMs - gLastConnectAttemptMs < 3000)
        {
            return;
        }
        gLastConnectAttemptMs = nowMs;

        // 每次重连都重新生成 token，降低 token 过期导致登录失败的风险
        String token = generateOneNetToken(
            gConfig.base64Key,
            buildResource(),
            gConfig.tokenExpireAt,
            gConfig.signMethod,
            "2018-10-31");

        if (token.isEmpty())
        {
            // Serial.println("[OneNET] token generate failed");
            return;
        }

        // Serial.println("[OneNET] MQTT connecting...");
        bool ok = gMqttClient.connect(gConfig.deviceName, gConfig.productId, token.c_str());
        if (!ok)
        {
            // Serial.print("[OneNET] MQTT connect failed, rc=");
            // Serial.println(gMqttClient.state());
            return;
        }

        // Serial.println("[OneNET] MQTT connected");

        // 订阅平台应答与属性下发主题
        gMqttClient.subscribe(gPropertyPostReplyTopic.c_str(), 0);
        gMqttClient.subscribe(gPropertySetTopic.c_str(), 0);
    }
}

/**
 * @brief 配置并初始化 OneNET MQTT 模块
 * @param config OneNET 连接配置（host、port、productId、deviceName、base64Key、tokenExpireAt、signMethod）
 *
 * 说明：
 * 1. 保存配置并生成物模型标准 Topic 字符串缓存。
 * 2. 配置 MQTT 客户端的服务器地址、消息回调及发送缓冲区大小。
 * 3. 本函数只做配置，真实连接延后到 oneNetMqttLoop() 自动完成。
 */
void oneNetMqttBegin(const OneNetMqttConfig &config)
{
    gConfig = config;

    // OneNET 物模型标准 Topic
    gPropertyPostTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/post";
    gPropertyPostReplyTopic = gPropertyPostTopic + "/reply";
    gPropertySetTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/set";

    gMqttClient.setServer(gConfig.host, gConfig.port);
    gMqttClient.setCallback(mqttCallback);
    // 扩大发送缓冲区，9 个属性的 payload 约 380 字节，默认 256 不够
    gMqttClient.setBufferSize(512);

    gInited = true;
}

/**
 * @brief 维护 MQTT 连接与消息收发（在 loop() 中持续调用）
 *
 * 说明：
 * 1. 模块未初始化时直接返回。
 * 2. 断开时调用 tryConnect() 尝试重连（内部有 3s 限流）。
 * 3. 已连接时调用 PubSubClient::loop() 处理保活和下行消息。
 */
void oneNetMqttLoop()
{
    if (!gInited)
    {
        return;
    }

    if (!gMqttClient.connected())
    {
        tryConnect();
        return;
    }

    // 已连接时持续处理保活和收发
    gMqttClient.loop();
}

/**
 * @brief 查询 MQTT 是否处于已连接状态
 * @return true 已连接；false 未连接或模块未初始化
 */
bool oneNetMqttConnected()
{
    return gMqttClient.connected();
}

/**
 * @brief 向 OneNET 平台上报三仓物模型属性
 * @param phone   手机仓数据（重量、百分比、满溢标志）
 * @param mouse   鼠标仓数据（重量、百分比、满溢标志）
 * @param battery 电池仓数据（重量、百分比、满溢标志）
 * @return true 发布成功；false MQTT 未连接或发布失败
 *
 * 说明：
 * 按 OneJSON 格式组装 9 个属性的 payload（约 380 字节），
 * 发布至物模型属性上报 Topic。float 使用 dtostrf 格式化，
 * 发送缓冲区已在 oneNetMqttBegin() 中扩大至 512 字节。
 */
bool oneNetMqttUploadProperties(
    const BoxBinData &phone,
    const BoxBinData &mouse,
    const BoxBinData &battery)
{
    if (!gMqttClient.connected())
    {
        return false;
    }

    String msgId = buildMessageId();

    // float 格式化为字符串（dtostrf 是 AVR/ESP32 标准函数）
    char phonePct[10], mousePct[10], batteryPct[10];
    dtostrf(phone.percent, 1, 2, phonePct);
    dtostrf(mouse.percent, 1, 2, mousePct);
    dtostrf(battery.percent, 1, 2, batteryPct);

    // OneJSON 属性上报载荷，包含全部 9 个物模型属性
    // payload 最大约 380 字节，缓冲区 512 足够
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
             "\"phone_weight\":{\"value\":%d},"
             "\"phone_percent\":{\"value\":%s},"
             "\"phone_full\":{\"value\":%s},"
             "\"mouse_weight\":{\"value\":%d},"
             "\"mouse_percent\":{\"value\":%s},"
             "\"mouse_full\":{\"value\":%s},"
             "\"battery_weight\":{\"value\":%d},"
             "\"battery_percent\":{\"value\":%s},"
             "\"battery_full\":{\"value\":%s}"
             "}}",
             msgId.c_str(),
             phone.weight, phonePct, phone.full ? "true" : "false",
             mouse.weight, mousePct, mouse.full ? "true" : "false",
             battery.weight, batteryPct, battery.full ? "true" : "false");

    bool ok = gMqttClient.publish(gPropertyPostTopic.c_str(), payload, false);
    // if (ok)
    // {
    //     Serial.print("[OneNET] property post: ");
    //     Serial.println(payload);
    // }
    // else
    // {
    //     Serial.println("[OneNET] property post failed");
    // }
    return ok;
}
