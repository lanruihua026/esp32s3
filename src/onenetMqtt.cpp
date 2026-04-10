#include "onenetMqtt.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

namespace
{
    // ===== OneNET MQTT 模块内部状态 =====
    // 这里封装的是“联网后把三路垃圾桶数据上传到云平台”的能力。

    // MQTT 传输底层客户端。
    // 当前使用普通 TCP，如果后续需要加密通信，可替换成 WiFiClientSecure。
    WiFiClient gWifiClient;
    PubSubClient gMqttClient(gWifiClient);

    // 启动时写入的 OneNET 配置。
    // 实际值由 main.cpp 的 oneNetMqttBegin() 覆盖，此处仅为结构体默认占位。
    OneNetMqttConfig gConfig = {
        "mqtts.heclouds.com",
        1883,
        "",
        "",
        "",
        0, // tokenExpireAt 占位，由 oneNetMqttBegin() 传入 main.cpp 的 ONENET_TOKEN_EXPIRE_AT
        OneNetSignMethod::SHA256};

    // 是否已经完成参数配置。
    bool gInited = false;

    // 最近一次重连尝试时间，用来避免频繁重连。
    uint32_t gLastConnectAttemptMs = 0;

    // 平台属性下发（property/set）用户回调，nullptr 表示未注册。
    static void (*gPropertySetCb)(const char *payload, unsigned int len) = nullptr;

    // OneNET 物模型相关 Topic 缓存，避免每次上报都重新拼接字符串。
    String gPropertyPostTopic;
    String gPropertyPostReplyTopic;
    String gPropertySetTopic;
    String gPropertySetReplyTopic; // 设备收到 property/set 后必须在此 topic 回复，否则平台视为命令超时

    /**
     * @brief 生成 OneNET token 所需的资源串
     * 业务含义：标识“哪个产品下的哪台设备”。
     */
    String buildResource()
    {
        return String("products/") + gConfig.productId + "/devices/" + gConfig.deviceName;
    }

    /**
     * @brief 生成消息 ID
     * 业务含义：每次属性上报都带一个唯一编号，便于平台区分消息。
     */
    String buildMessageId()
    {
        return String(static_cast<uint32_t>(millis()));
    }

    /**
     * @brief 平台下行消息回调
     *
     * 收到 thing/property/set 时：
     * 1. 尽快向 thing/property/set_reply 发布应答
     *    ——这是 OneNET 物模型协议的强制要求：
     *    若设备不回复，平台将该命令标记为超时，后续下发的命令也会被阻塞。
     * 2. 再调用用户注册的处理回调（解析 JSON、更新阈值等）
     */
    void mqttCallback(char *topic, byte *payload, unsigned int length)
    {
        if (gPropertySetTopic.length() == 0 || strcmp(topic, gPropertySetTopic.c_str()) != 0)
        {
            return; // 不是 property/set 消息，忽略
        }

        char payloadCopy[512] = {0};
        const unsigned int copyLen = (length < sizeof(payloadCopy) - 1) ? length : sizeof(payloadCopy) - 1;
        if (copyLen > 0)
        {
            memcpy(payloadCopy, payload, copyLen);
        }

        // === OneNET 协议：必须尽快回复 set_reply ===
        // 从 payload 中提取消息 id，用于回复中与请求配对。
        // payload 格式：{"id":"xxx","version":"1.0","params":{...}}
        // 注意：mqttCallback 由 PubSubClient::loop() 在主循环中调用，并非中断上下文；
        //       此处用 ArduinoJson 解析 id 字段，保证与平台 payload 格式无关。
        char msgId[32] = "0";
        {
            // 仅解析顶层 id 字段，使用小容量文档避免不必要的内存占用。
            JsonDocument idDoc;
            DeserializationError err = deserializeJson(idDoc, payloadCopy, copyLen);
            if (err == DeserializationError::Ok)
            {
                const char *parsedId = idDoc["id"] | "0";
                strncpy(msgId, parsedId, sizeof(msgId) - 1);
                msgId[sizeof(msgId) - 1] = '\0';
            }
            else
            {
                Serial.printf("[OneNET] SET_REPLY id parse err: %s\n", err.c_str());
            }
        }

        char replyBuf[96];
        snprintf(replyBuf, sizeof(replyBuf),
                 "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\"}", msgId);
        bool sent = gMqttClient.publish(gPropertySetReplyTopic.c_str(), replyBuf, false);
        Serial.printf("[OneNET] SET_REPLY id=%s sent=%d\n", msgId, sent ? 1 : 0);

        // 回执发出后再执行业务逻辑，避免 NVS 写入/属性补上报拖慢平台 ACK。
        if (gPropertySetCb != nullptr)
        {
            gPropertySetCb(payloadCopy, copyLen);
        }
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

        // 每次重连都重新生成 token，避免 token 过期后还拿旧值去登录。
        String token = generateOneNetToken(
            gConfig.base64Key,
            buildResource(),
            gConfig.tokenExpireAt,
            gConfig.signMethod,
            "2018-10-31");

        if (token.isEmpty())
        {
            Serial.println("[OneNET] TOKEN FAIL");
            return;
        }

        bool ok = gMqttClient.connect(gConfig.deviceName, gConfig.productId, token.c_str());
        if (!ok)
        {
            Serial.printf("[OneNET] MQTT FAIL rc=%d\n", gMqttClient.state());
            return;
        }

        Serial.println("[OneNET] MQTT OK");

        // 连接成功后订阅平台应答和属性下发主题。
        // QoS 1：保证至少收到一次，防止平台下发指令因网络抖动丢失。
        bool subReply = gMqttClient.subscribe(gPropertyPostReplyTopic.c_str(), 1);
        bool subSet   = gMqttClient.subscribe(gPropertySetTopic.c_str(), 1);
        Serial.printf("[OneNET] SUB post/reply=%d  property/set=%d\n",
                      subReply ? 1 : 0, subSet ? 1 : 0);
    }
}

/**
 * @brief 配置并初始化 OneNET MQTT 模块
 * @param config OneNET 连接配置（host、port、productId、deviceName、base64Key、tokenExpireAt、signMethod）
 *
 * 说明：
 * 1. 保存云平台连接参数。
 * 2. 生成后续属性上报要用到的 Topic。
 * 3. 配置 MQTT 客户端，但不在这里阻塞等待联网成功。
 */
void oneNetMqttBegin(const OneNetMqttConfig &config)
{
    gConfig = config;

    // OneNET 物模型标准 Topic。
    gPropertyPostTopic      = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/post";
    gPropertyPostReplyTopic = gPropertyPostTopic + "/reply";
    gPropertySetTopic       = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/set";
    gPropertySetReplyTopic  = gPropertySetTopic + "_reply"; // 设备回复平台下发的属性设置命令

    gMqttClient.setServer(gConfig.host, gConfig.port);
    gMqttClient.setCallback(mqttCallback);
    // 现在上报 11 个属性（含 overflow_threshold_g、ai_conf_threshold），适当扩容缓冲区。
    gMqttClient.setBufferSize(768);
    // 弱网/离线时缩短单次阻塞，避免长时间卡住 loop（秒；与 tryConnect 3s 节流配合）
    // 优化：5秒→3秒，局域网/云端MQTT环境下3秒超时充足
    gMqttClient.setSocketTimeout(3);

    gInited = true;
}

/**
 * @brief 维护 MQTT 连接与消息收发（在 loop() 中持续调用）
 *
 * 说明：
 * 1. 未初始化时不做任何事。
 * 2. 未连接时尝试重连。
 * 3. 已连接时处理保活和平台消息。
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

    // 已连接时持续处理保活和收发。
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
 * @brief 向 OneNET 平台上报三仓物模型属性及阈值
 * @param phone   手机仓数据（重量、百分比、满溢标志）
 * @param mouse   鼠标仓数据（重量、百分比、满溢标志）
 * @param battery 电池仓数据（重量、百分比、满溢标志）
 * @param overflowThresholdG 满载阈值（克）
 * @param aiConfThreshold AI 置信度阈值（0~1）
 * @return true 发布成功；false MQTT 未连接或发布失败
 */
bool oneNetMqttUploadProperties(
    const BoxBinData &phone,
    const BoxBinData &mouse,
    const BoxBinData &battery,
    int32_t overflowThresholdG,
    float aiConfThreshold)
{
    if (!gMqttClient.connected())
    {
        return false;
    }

    String msgId = buildMessageId();

    // 百分比是 float，需要先转成字符串再拼进 JSON。
    char phonePct[10], mousePct[10], batteryPct[10];
    dtostrf(phone.percent, 1, 2, phonePct);
    dtostrf(mouse.percent, 1, 2, mousePct);
    dtostrf(battery.percent, 1, 2, batteryPct);
    char aiConfStr[16];
    dtostrf(aiConfThreshold, 1, 4, aiConfStr);

    // OneJSON 属性上报载荷：三仓属性（重量/百分比/即将满载/满溢）+ 满载阈值 + AI 置信度阈值，共 14 个属性。
    char payload[840];
    snprintf(payload, sizeof(payload),
             "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
             "\"phone_weight\":{\"value\":%d},"
             "\"phone_percent\":{\"value\":%s},"
             "\"phone_near_full\":{\"value\":%s},"
             "\"phone_full\":{\"value\":%s},"
             "\"mouse_weight\":{\"value\":%d},"
             "\"mouse_percent\":{\"value\":%s},"
             "\"mouse_near_full\":{\"value\":%s},"
             "\"mouse_full\":{\"value\":%s},"
             "\"battery_weight\":{\"value\":%d},"
             "\"battery_percent\":{\"value\":%s},"
             "\"battery_near_full\":{\"value\":%s},"
             "\"battery_full\":{\"value\":%s},"
             "\"overflow_threshold_g\":{\"value\":%d},"
             "\"ai_conf_threshold\":{\"value\":%s}"
             "}}",
             msgId.c_str(),
             phone.weight, phonePct, phone.nearFull ? "true" : "false", phone.full ? "true" : "false",
             mouse.weight, mousePct, mouse.nearFull ? "true" : "false", mouse.full ? "true" : "false",
             battery.weight, batteryPct, battery.nearFull ? "true" : "false", battery.full ? "true" : "false",
             overflowThresholdG,
             aiConfStr);

    bool ok = gMqttClient.publish(gPropertyPostTopic.c_str(), payload, false);
    Serial.println(ok ? "[OneNET] POST OK" : "[OneNET] POST FAIL");
    return ok;
}

/**
 * @brief 注册平台属性下发（property/set）回调
 * @param cb 回调函数指针；传 nullptr 取消注册
 */
void oneNetSetPropertySetCallback(void (*cb)(const char *payload, unsigned int len))
{
    gPropertySetCb = cb;
}

/**
 * @brief 主动断开 MQTT 连接
 *
 * 在 WiFi 掉线时由主循环调用，防止 TCP 半开状态导致后续上报静默失败。
 * 断开后 oneNetMqttLoop() 会在 WiFi 恢复时自动重连。
 */
void oneNetMqttDisconnect()
{
    if (gMqttClient.connected())
    {
        gMqttClient.disconnect();
        Serial.println("[OneNET] MQTT disconnect (WiFi lost)");
    }
}
