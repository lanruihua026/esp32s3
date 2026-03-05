#ifndef _ONENET_TOKEN_H_
#define _ONENET_TOKEN_H_

#include <Arduino.h>

/**
 * @brief OneNET token 支持的签名算法
 *
 * 对应 OneNET 文档中的 method 字段：
 * - md5
 * - sha1
 * - sha256
 */
enum class OneNetSignMethod
{
    SHA1,
    SHA256,
    MD5
};

/**
 * @brief 将签名算法枚举转换为协议字符串
 */
String oneNetSignMethodToString(OneNetSignMethod method);

/**
 * @brief 生成 OneNET 鉴权 token
 *
 * @param base64Key 平台下发的 Base64 编码设备密钥
 * @param resource 资源串（通常为 products/{pid}/devices/{deviceName}）
 * @param expirationTime 过期时间（Unix 秒级时间戳）
 * @param method 签名算法，默认 SHA256
 * @param version token 版本，默认 2018-10-31
 * @return 生成成功返回 token，失败返回空字符串
 */
String generateOneNetToken(
    const String &base64Key,
    const String &resource,
    uint32_t expirationTime,
    OneNetSignMethod method = OneNetSignMethod::SHA256,
    const String &version = "2018-10-31");

#endif
