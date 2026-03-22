#include "onenetMqtt.h"
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
    OneNetMqttConfig gConfig = {
        "mqtts.heclouds.com",
        1883,
        "",
        "",
        "",
        1893456000,
        OneNetSignMethod::SHA256};

    // 是否已经完成参数配置。
    bool gInited = false;

    // 最近一次重连尝试时间，用来避免频繁重连。
    uint32_t gLastConnectAttemptMs = 0;

    // OneNET 物模型相关 Topic 缓存，避免每次上报都重新拼接字符串。
    String gPropertyPostTopic;
    String gPropertyPostReplyTopic;
    String gPropertySetTopic;

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
     * 当前项目只保留接口，不主动处理平台下发控制命令。
     */
    void mqttCallback(char *topic, byte *payload, unsigned int length)
    {
        (void)topic;
        (void)payload;
        (void)length;
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

        // 连接成功后订阅平台应答和属性下发主题。
        gMqttClient.subscribe(gPropertyPostReplyTopic.c_str(), 0);
        gMqttClient.subscribe(gPropertySetTopic.c_str(), 0);
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
    gPropertyPostTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/post";
    gPropertyPostReplyTopic = gPropertyPostTopic + "/reply";
    gPropertySetTopic = String("$sys/") + gConfig.productId + "/" + gConfig.deviceName + "/thing/property/set";

    gMqttClient.setServer(gConfig.host, gConfig.port);
    gMqttClient.setCallback(mqttCallback);
    // 三个仓位一共要上报 9 个属性，默认缓冲区偏小，这里提前扩容。
    gMqttClient.setBufferSize(512);

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
 * @brief 向 OneNET 平台上报三仓物模型属性
 * @param phone   手机仓数据（重量、百分比、满溢标志）
 * @param mouse   鼠标仓数据（重量、百分比、满溢标志）
 * @param battery 电池仓数据（重量、百分比、满溢标志）
 * @return true 发布成功；false MQTT 未连接或发布失败
 *
 * 说明：
 * 业务含义：
 * 把三个仓位的重量、百分比、满载状态统一打包，
 * 以上报到 OneNET 物模型，供云端页面或小程序查看。
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

    // 百分比是 float，需要先转成字符串再拼进 JSON。
    char phonePct[10], mousePct[10], batteryPct[10];
    dtostrf(phone.percent, 1, 2, phonePct);
    dtostrf(mouse.percent, 1, 2, mousePct);
    dtostrf(battery.percent, 1, 2, batteryPct);

    // OneJSON 属性上报载荷，包含三个仓位的全部属性。
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
    return ok;
}
