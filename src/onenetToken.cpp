#include "onenetToken.h"

#include <mbedtls/base64.h>
#include <mbedtls/md.h>

namespace
{
    /**
     * @brief Base64 解码
     */
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

    /**
     * @brief Base64 编码
     *
     * mbedTLS 的推荐用法：
     * 1. 先调用一次拿到所需长度；
     * 2. 再分配缓冲区执行真实编码。
     */
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

    /**
     * @brief 对 token 中的 value 做 URL 编码
     *
     * OneNET token 的各字段值需要 URL 编码，
     * 例如 '/' -> %2F, '=' -> %3D。
     */
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

    /**
     * @brief 将项目内签名算法映射到 mbedTLS 摘要算法
     */
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

/**
 * @brief 将签名算法枚举转换为 OneNET token 所需的字符串表示
 * @param method 签名算法枚举值（MD5 / SHA1 / SHA256）
 * @return 对应的小写算法名称字符串（"md5" / "sha1" / "sha256"）；
 *         未知值默认返回 "sha256"
 */
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

/**
 * @brief 生成符合 OneNET 鉴权规范的 token 字符串
 * @param base64Key      Base64 编码的设备密钥
 * @param resource       资源串，格式为 "products/<pid>/devices/<dname>"
 * @param expirationTime Token 过期时间（Unix 时间戳，秒）
 * @param method         签名算法（MD5 / SHA1 / SHA256）
 * @param version        协议版本字符串（如 "2018-10-31"）
 * @return 组装好的 token 字符串；参数非法或签名失败时返回空字符串
 *
 * 说明：
 * 签名流程：
 * 1. Base64 解码密钥；
 * 2. 按固定顺序拼接 StringForSignature（et\\nmethod\\nres\\nversion）；
 * 3. 使用 mbedTLS HMAC 计算摘要；
 * 4. 将摘要再 Base64 编码得到 sign；
 * 5. 对各字段值做 URL 编码后拼接为最终 token。
 */
String generateOneNetToken(
    const String &base64Key,
    const String &resource,
    uint32_t expirationTime,
    OneNetSignMethod method,
    const String &version)
{
    // 参数兜底校验，避免进入签名流程后才失败
    if (base64Key.isEmpty() || resource.isEmpty())
    {
        return "";
    }

    // OneNET 规则：先 Base64 解码密钥，再进行 HMAC
    uint8_t decodedKey[128] = {0};
    size_t keyLen = 0;
    if (!base64Decode(base64Key, decodedKey, sizeof(decodedKey), keyLen) || keyLen == 0)
    {
        return "";
    }

    // StringForSignature 固定顺序：et + '\n' + method + '\n' + res + '\n' + version
    String methodStr = oneNetSignMethodToString(method);
    String et = String(expirationTime);
    String stringForSignature = et + "\n" + methodStr + "\n" + resource + "\n" + version;

    const mbedtls_md_info_t *mdInfo = getMdInfo(method);
    if (mdInfo == nullptr)
    {
        return "";
    }

    // 计算 HMAC
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

    // HMAC 结果再做 Base64，得到 sign 原始值
    size_t hmacLen = mbedtls_md_get_size(mdInfo);
    String signBase64;
    if (!base64Encode(hmacOutput, hmacLen, signBase64))
    {
        return "";
    }

    // 组装 token（每个字段值都要 URL 编码）
    String token = "version=" + urlEncodeValue(version) +
                   "&res=" + urlEncodeValue(resource) +
                   "&et=" + urlEncodeValue(et) +
                   "&method=" + urlEncodeValue(methodStr) +
                   "&sign=" + urlEncodeValue(signBase64);

    return token;
}
