#ifndef HX711_BOOT_CONFIG_H
#define HX711_BOOT_CONFIG_H

#include <cstdint>

/**
 * HX711 启动预热策略（与称重稳定性相关，改前请真机对比首分钟漂移）
 *
 * 0 = 保守：每路 setup 内各等待 HX711_PER_CHANNEL_WARMUP_MS（与历史行为一致）
 * 1 = 快速：仅在 initHx711Modules() 开头等待 HX711_GLOBAL_WARMUP_MS，各路 setup 内不再等待
 */
constexpr int HX711_BOOT_STRATEGY = 0;

constexpr uint32_t HX711_GLOBAL_WARMUP_MS = 1200;
constexpr uint32_t HX711_PER_CHANNEL_WARMUP_MS = 1200;

#endif
