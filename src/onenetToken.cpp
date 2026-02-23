#include "onenetToken.h"

#include <mbedtls/base64.h>
#include <mbedtls/md.h>

namespace
{
    // Base64 解码工具函数。
    // 将输入字符串解码到 output 缓冲区，并返回实际解码长度 decodedLen。
    bool base64Decode(const String &input, uint8_t *output, size_t outputSize, size_t &decodedLen)
    {
        int ret = mbedtls_base64_decode(
            output,
            outputSize,
            &decodedLen,
            reinterpret_cast<const uint8_t *>(input.c_str()),
            input.length());
        return ret == 0;
    }

    // Base64 编码工具函数。
    // 分两步完成：先获取所需长度，再分配缓冲区进行正式编码。
    bool base64Encode(const uint8_t *input, size_t inputLen, String &output)
    {
        size_t requiredLen = 0;
        int ret = mbedtls_base64_encode(nullptr, 0, &requiredLen, input, inputLen);
        if (ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL || requiredLen == 0)
        {
            return false;
        }

        char *buffer = new char[requiredLen + 1];
        if (buffer == nullptr)
        {
            return false;
        }

        size_t actualLen = 0;
        ret = mbedtls_base64_encode(
            reinterpret_cast<uint8_t *>(buffer),
            requiredLen,
            &actualLen,
            input,
            inputLen);

        if (ret != 0)
        {
            delete[] buffer;
            return false;
        }

        buffer[actualLen] = '\0';
        output = String(buffer);
        delete[] buffer;
        return true;
    }

    // 对 token 中 value 做 URL 编码。
    // OneNET 文档要求 token 的 value 部分必须进行 URL 编码，
    // 例如 '/' -> %2F, '=' -> %3D。
    String urlEncodeValue(const String &value)
    {
        String encoded;
        encoded.reserve(value.length() * 3);

        const char *hex = "0123456789ABCDEF";
        for (size_t i = 0; i < value.length(); ++i)
        {
            uint8_t c = static_cast<uint8_t>(value[i]);
            bool unreserved =
                (c >= 'A' && c <= 'Z') ||
                (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') ||
                c == '-' || c == '_' || c == '.' || c == '~';

            if (unreserved)
            {
                encoded += static_cast<char>(c);
            }
            else
            {
                encoded += '%';
                encoded += hex[(c >> 4) & 0x0F];
                encoded += hex[c & 0x0F];
            }
        }

        return encoded;
    }

    // 根据签名方法映射到 mbedTLS 的消息摘要算法描述。
    const mbedtls_md_info_t *getMdInfo(OneNetSignMethod method)
    {
        switch (method)
        {
        case OneNetSignMethod::MD5:
            return mbedtls_md_info_from_type(MBEDTLS_MD_MD5);
        case OneNetSignMethod::SHA1:
            return mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
        case OneNetSignMethod::SHA256:
        default:
            return mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        }
    }
}

// 将内部枚举转为协议字符串。
String oneNetSignMethodToString(OneNetSignMethod method)
{
    switch (method)
    {
    case OneNetSignMethod::MD5:
        return "md5";
    case OneNetSignMethod::SHA1:
        return "sha1";
    case OneNetSignMethod::SHA256:
    default:
        return "sha256";
    }
}

String generateOneNetToken(
    const String &base64Key,
    const String &resource,
    uint32_t expirationTime,
    OneNetSignMethod method,
    const String &version)
{
    // 基础参数校验，避免进入后续签名流程后才报错。
    if (base64Key.isEmpty() || resource.isEmpty())
    {
        return "";
    }

    // OneNET 要求先将 key 做 Base64 解码，再参与 HMAC 计算。
    uint8_t decodedKey[128] = {0};
    size_t keyLen = 0;
    if (!base64Decode(base64Key, decodedKey, sizeof(decodedKey), keyLen) || keyLen == 0)
    {
        return "";
    }

    // 按文档固定顺序构造 StringForSignature：
    // et + '\n' + method + '\n' + res + '\n' + version
    String methodStr = oneNetSignMethodToString(method);
    String et = String(expirationTime);
    String stringForSignature = et + "\n" + methodStr + "\n" + resource + "\n" + version;

    // 获取 HMAC 使用的摘要算法。
    const mbedtls_md_info_t *mdInfo = getMdInfo(method);
    if (mdInfo == nullptr)
    {
        return "";
    }

    // 计算 HMAC 结果。
    uint8_t hmacOutput[32] = {0};
    int hmacRet = mbedtls_md_hmac(
        mdInfo,
        decodedKey,
        keyLen,
        reinterpret_cast<const unsigned char *>(stringForSignature.c_str()),
        stringForSignature.length(),
        hmacOutput);
    if (hmacRet != 0)
    {
        return "";
    }

    // 对 HMAC 结果再次做 Base64，得到 sign 原始值。
    size_t hmacLen = mbedtls_md_get_size(mdInfo);
    String signBase64;
    if (!base64Encode(hmacOutput, hmacLen, signBase64))
    {
        return "";
    }

    // 组装最终 token。
    // 注意：每个字段的 value 都必须 URL 编码。
    String token = "version=" + urlEncodeValue(version) +
                   "&res=" + urlEncodeValue(resource) +
                   "&et=" + urlEncodeValue(et) +
                   "&method=" + urlEncodeValue(methodStr) +
                   "&sign=" + urlEncodeValue(signBase64);

    return token;
}