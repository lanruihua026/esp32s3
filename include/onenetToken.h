#ifndef _ONENET_TOKEN_H_
#define _ONENET_TOKEN_H_

#include <Arduino.h>

enum class OneNetSignMethod
{
    SHA1,
    SHA256,
    MD5
};

String oneNetSignMethodToString(OneNetSignMethod method);

String generateOneNetToken(
    const String &base64Key,
    const String &resource,
    uint32_t expirationTime,
    OneNetSignMethod method = OneNetSignMethod::SHA256,
    const String &version = "2018-10-31");

#endif