#ifndef _ONENET_TOKEN_H_
#define _ONENET_TOKEN_H_

#include <Arduino.h>

// OneNET Token 支持的签名算法。
// 与平台文档中的 method 字段对应：md5 / sha1 / sha256。
enum class OneNetSignMethod
{
    SHA1,
    SHA256,
    MD5
};

// 将签名算法枚举转换为 OneNET 协议要求的 method 字符串。
String oneNetSignMethodToString(OneNetSignMethod method);

// 生成 OneNET 鉴权 token。
// 参数说明：
// - base64Key: 平台分配的密钥（注意：必须是 Base64 编码字符串）。
// - resource: 资源串，通常为 products/{pid}/devices/{deviceName}。
// - expirationTime: 过期时间（Unix 秒级时间戳）。
// - method: 签名算法，默认使用 SHA256。
// - version: token 版本号，当前常用 2018-10-31。
// 返回值：
// - 成功返回完整 token 字符串。
// - 失败返回空字符串。
String generateOneNetToken(
    const String &base64Key,
    const String &resource,
    uint32_t expirationTime,
    OneNetSignMethod method = OneNetSignMethod::SHA256,
    const String &version = "2018-10-31");

#endif