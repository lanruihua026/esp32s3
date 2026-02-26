#include "onenetMqtt.h"

#include <PubSubClient.h>
#include <WiFi.h>

namespace
{
    // MQTT 传输底层 TCP 客户端（非 TLS 版本）。
    // 若后续需要 TLS，可替换为 WiFiClientSecure。
    WiFiClient gWifiClient;

    // PubSubClient 负责 MQTT 协议处理。
    PubSubClient gMqttClient(gWifiClient);

    // 默认配置占位值，会在 oneNetMqttBegin 中被用户配置覆盖。
    OneNetMqttConfig gConfig = {
        "mqtts.heclouds.com",
        1883,
        "",
        "",
        "",
        1893456000,
        OneNetSignMethod::SHA256};

    // 模块初始化标记。
    bool gInited = false;

    // 最近一次连接尝试时间，用于限流重连（避免过于频繁）。
    uint32_t gLastConnectAttemptMs = 0;

    // OneNET 相关 Topic 缓存。
    // - 上报属性 Topic
    // - 上报属性应答 Topic
    // - 属性设置下发 Topic
    String gPropertyPostTopic;
    String gPropertyPostReplyTopic;
    String gPropertySetTopic;

    // 构造 token 使用的资源串：products/{pid}/devices/{deviceName}
    String buildResource()
    {
        return String("products/") + gConfig.productId + "/devices/" + gConfig.deviceName;
    }

    // 生成 OneJSON 的 id 字段。
    // 这里使用 millis 作为简单消息序号。
    String buildMessageId()
    {
        return String(static_cast<uint32_t>(millis()));
    }

    // MQTT 订阅回调。
    // 当前实现先统一打印，便于调试平台回复与属性下发。
    void mqttCallback(char *topic, byte *payload, unsigned int length)
    {
        String msg;
        msg.reserve(length);
        for (unsigned int i = 0; i < length; ++i)
        {
            msg += static_cast<char>(payload[i]);
        }

        Serial.print("[OneNET] topic: ");
        Serial.println(topic);
        Serial.print("[OneNET] payload: ");
        Serial.println(msg);
    }

    // 尝试建立 MQTT 连接。
    // 连接条件：
    // 1) 已初始化
    // 2) WiFi 已连接
    // 3) MQTT 当前未连接
    // 4) 距离上次尝试超过 3 秒
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

        // 每次连接前重新生成 token，避免 token 过期导致登录失败。
        String token = generateOneNetToken(
            gConfig.base64Key,
            buildResource(),
            gConfig.tokenExpireAt,
            gConfig.signMethod,
            "2018-10-31");

        if (token.isEmpty())
        {
            Serial.println("[OneNET] token generate failed");
            return;
        }

        Serial.println("[OneNET] MQTT connecting...");
        bool ok = gMqttClient.connect(gConfig.deviceName, gConfig.productId, token.c_str());
        if (!ok)
        {
            Serial.print("[OneNET] MQTT connect failed, rc=");
            Serial.println(gMqttClient.state());
            return;
        }

        Serial.println("[OneNET] MQTT connected");

        // 订阅平台应答与下发主题。
        // - property/post/reply: 上报后平台回执
        // - property/set: 平台属性下发
        gMqttClient.subscribe(gPropertyPostReplyTopic.c_str(), 0);
        gMqttClient.subscribe(gPropertySetTopic.c_str(), 0);
    }
}

void oneNetMqttBegin(const OneNetMqttConfig &config)
{
    // 保存配置到全局上下文。
    gConfig = config;

    // 组装 OneNET 标准 Topic。
    gPropertyPostTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/post";
    gPropertyPostReplyTopic = gPropertyPostTopic + "/reply";
    gPropertySetTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/set";

    // 配置 MQTT 服务地址和消息回调。
    gMqttClient.setServer(gConfig.host, gConfig.port);
    gMqttClient.setCallback(mqttCallback);

    gInited = true;
}

void oneNetMqttLoop()
{
    // 若未初始化，直接返回。
    if (!gInited)
    {
        return;
    }

    // 未连接时执行重连逻辑。
    if (!gMqttClient.connected())
    {
        tryConnect();
        return;
    }

    // 已连接时执行 MQTT 协议收发维护。
    gMqttClient.loop();
}

bool oneNetMqttConnected()
{
    return gMqttClient.connected();
}

bool oneNetMqttUploadProperties(bool isFull, int32_t weight)
{
    // 仅在连接成功后上报。
    if (!gMqttClient.connected())
    {
        return false;
    }

    // OneJSON 公共字段。
    String msgId = buildMessageId();
    // 组装 OneJSON 属性上报数据。
    // 文档格式：
    // {
    //   "id":"xxx",
    //   "version":"1.0",
    //   "params":{
    //      "identifier":{ "value":..., "time":... }
    //   }
    // }
    String payload;
    payload.reserve(256);
    payload += "{\"id\":\"" + msgId + "\",\"version\":\"1.0\",\"params\":{";
    payload += "\"isFull\":{\"value\":";
    payload += (isFull ? "true" : "false");
    payload += "},";
    payload += "\"weight\":{\"value\":" + String(weight) + "}";
    payload += "}}";

    // 发布到 OneNET 物模型属性上报 Topic。
    bool ok = gMqttClient.publish(gPropertyPostTopic.c_str(), payload.c_str(), false);
    if (ok)
    {
        Serial.print("[OneNET] property post: ");
        Serial.println(payload);
    }
    else
    {
        Serial.println("[OneNET] property post failed");
    }
    return ok;
}