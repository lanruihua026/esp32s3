#include "hx711_2.h"
#include "hx711BootConfig.h"

#include <math.h>

// ===== 第二个 HX711 引脚定义 =====
#define HX711_DT2 11
#define HX711_SCK2 12

// ===== 称重参数 =====
// calibration_factor2：每 1 克对应的原始 ADC 差值（raw / g），始终为正
static float calibration_factor2 = 1000.0f;
// zero_offset2：去皮后的零点偏移（原始 ADC），由 tareScale2() 采样设置
static float zero_offset2 = 0.0f;

// 初始化完成标志，false 时 getWeight2() 直接返回 0
static bool isScaleReady2 = false;

// 部分接线或受力方向下，施加重量可能使原始值变小（负方向）。
// scale_direction2 用于统一把"加重"映射为正重量：+1 表示正向，-1 表示反向。
static int8_t scale_direction2 = 1;
// 一旦检测到足够明显的受力变化，就锁定方向，避免来回抖动
static bool direction_locked2 = false;

// 小重量死区：将非常小的噪声视为 0g，提升静止显示稳定性
static const float ZERO_DEADBAND_G2 = 2.0f;
// 方向锁定阈值：净原始值绝对值超过该值时才判断并锁定受力方向
static const float DIRECTION_LOCK_RAW_THRESHOLD2 = 30000.0f;
// 去皮时要求的最小有效样本数（共采 15 帧）
static const int TARE_MIN_VALID_SAMPLES2 = 8;

// 前向声明，仅在本文件内部使用
static int32_t readRawData2();
static float readAverageRaw2(int samples, int *outValidCount = nullptr);

/**
 * @brief 等待第二个 HX711 数据就绪（DT 由高变低）
 * @param timeout_us 超时时间（微秒）
 * @return true 数据就绪；false 超时
 *
 * HX711 转换完成后会将 DT 拉低通知 MCU，此函数轮询该信号。
 * 期间调用 yield() 喂看门狗，防止长时间忙等触发 TWDT 重启。
 */
static bool waitDataReady2(uint32_t timeout_us)
{
    uint32_t start = micros();
    while (digitalRead(HX711_DT2) == HIGH)
    {
        if (micros() - start > timeout_us)
        {
            return false;
        }
        yield(); // 喂看门狗，防止长时间忙等触发 TWDT 重启
    }
    return true;
}

/**
 * @brief 初始化第二个 HX711 模块（不自动去皮）
 * 说明：
 * 1. 设置引脚模式，确保 SCK 初始为 LOW。
 * 2. 上电后等待 1.2 秒预热，降低初始漂移影响。
 * 3. 丢弃前 8 帧数据，进一步稳定读数。
 * 4. 通过 5 帧均值探测传感器是否正常响应（probeRaw == 0 视为超时失败）。
 * 5. 仅设置 isScaleReady2 标志，不执行去皮——零点由调用方从 NVS 恢复或在确认空仓后写入。
 * @return true 初始化成功；false 探测超时，传感器未就绪
 */
bool setupHX711_2()
{
    pinMode(HX711_DT2, INPUT);
    pinMode(HX711_SCK2, OUTPUT);
    digitalWrite(HX711_SCK2, LOW);
    if (HX711_BOOT_STRATEGY == 0)
    {
        delay(HX711_PER_CHANNEL_WARMUP_MS);
    }
    // 丢弃若干帧启动阶段的不稳定数据
    for (int i = 0; i < 8; ++i)
    {
        readRawData2();
    }

    // 通过一次均值采样判断传感器是否可用
    float probeRaw = readAverageRaw2(5);
    if (probeRaw == 0.0f)
    {
        isScaleReady2 = false;
        return false;
    }

    isScaleReady2 = true;
    // 不在此处去皮；零点由主流程从 NVS 恢复并按需条件去皮。
    return true;
}

/**
 * @brief 读取第二个 HX711 一帧 24 位原始值（增益 128，通道 A）
 * @return 带符号原始值（int32），等待超时时返回 0
 *
 * 时序说明：
 * - 循环产生 24 个 SCK 脉冲，每个脉冲上升沿读取 DT 上的一位数据，高位先出。
 * - 第 25 个额外脉冲用于向 HX711 写入下一次转换的增益/通道配置（此处保持 A 通道 128 增益）。
 * - 24 位数据以补码形式输出，最高位为符号位，需做符号扩展为 int32。
 */
static int32_t readRawData2()
{
    // 最长等待 100ms，避免阻塞过久
    if (!waitDataReady2(100000))
    {
        return 0;
    }

    uint32_t data = 0;

    // HX711 24 位数据，高位先出（MSB first）
    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(HX711_SCK2, HIGH);
        delayMicroseconds(1);

        if (digitalRead(HX711_DT2) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(HX711_SCK2, LOW);
        delayMicroseconds(1);
    }

    // 第 25 个时钟脉冲：保持 A 通道、增益 128
    digitalWrite(HX711_SCK2, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK2, LOW);
    delayMicroseconds(1);

    // 将 24 位补码扩展为 32 位有符号整数
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

/**
 * @brief 对第二个 HX711 多次采样求均值，自动剔除超时失败的帧
 * @param samples      采样次数（<=0 时直接返回 0）
 * @param outValidCount 输出参数：有效帧数量（可为 nullptr）
 * @return 有效样本的平均原始 ADC 值；全部失败时返回 0
 *
 * readRawData2() 超时返回 0，此处将其视为无效帧跳过，
 * 避免超时零值拉偏均值。
 */
static float readAverageRaw2(int samples, int *outValidCount)
{
    if (samples <= 0)
    {
        if (outValidCount != nullptr) *outValidCount = 0;
        return 0.0f;
    }

    int64_t sum = 0;       // 使用 int64 防止多帧累加溢出
    int validCount = 0;    // 有效（非超时）帧计数

    for (int i = 0; i < samples; i++)
    {
        int32_t raw = readRawData2();
        if (raw != 0)  // 0 视为超时失败帧，跳过
        {
            sum += raw;
            validCount++;
        }
    }

    if (outValidCount != nullptr) *outValidCount = validCount;

    if (validCount == 0)
    {
        return 0.0f;
    }

    return (float)sum / validCount;
}

/**
 * @brief 读取第二个 HX711 当前重量（单位：克）
 * @return 当前重量（g），未就绪或采样全部超时时返回 0
 *
 * 说明：
 * 1. 对多次采样均值进行去皮（减去 zero_offset2）得到净原始值。
 * 2. 首次净值超过方向锁定阈值时自动识别并锁定受力方向，统一让重量为正值。
 * 3. 小于死区阈值（ZERO_DEADBAND_G2）的抖动直接归零，提升静止显示稳定性。
 * 4. 业务上不允许负重量，若计算结果为负则截断为 0。
 */
float getWeight2()
{
    if (!isScaleReady2)
    {
        return 0.0f;
    }

    const int samples = 5;    // 每次读重量时的采样帧数
    int validCount = 0;
    float rawValue = readAverageRaw2(samples, &validCount);
    if (validCount == 0) return 0.0f;  // 全部超时，返回 0 避免误算
    float netValue = rawValue - zero_offset2;  // 减去零点偏移，得到净原始值

    // 自动识别受力方向并锁定：
    // 若受力后净值绝对值足够大且方向为负，后续统一乘 -1，让重量保持正向增长。
    if (!direction_locked2 && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD2)
    {
        scale_direction2 = (netValue >= 0.0f) ? 1 : -1;
        direction_locked2 = true;
    }

    // 净原始值乘方向系数后除以校准因子，得到克重
    float weight = (netValue * scale_direction2) / calibration_factor2;

    // 小抖动直接压到 0，提升静止显示稳定性
    if (fabsf(weight) < ZERO_DEADBAND_G2)
    {
        weight = 0.0f;
    }

    // 业务上不允许负重量
    if (weight < 0)
    {
        weight = 0;
    }

    return weight;
}

/**
 * @brief 获取第二个 HX711 当前载荷绝对值估计（克，不做方向裁剪）
 * @return >=0: 估计载荷；<0: 采样无效或未就绪
 */
float getLoadMagnitude2()
{
    if (!isScaleReady2)
    {
        return -1.0f;
    }

    int validCount = 0;
    float rawValue = readAverageRaw2(5, &validCount);
    if (validCount == 0)
    {
        return -1.0f;
    }

    float netValue = rawValue - zero_offset2;
    return fabsf(netValue) / calibration_factor2;
}

/**
 * @brief 使用已知砝码重量校准第二个传感器
 * @param knownWeight 已知砝码的实际重量（单位：克，必须 > 0）
 * @return true 校准成功并更新因子；false 校准失败（未就绪或有效样本不足）
 *
 * 说明：
 * 校准前请确保秤盘已去皮（调用过 tareScale2()）。
 * 函数通过多次采样计算新的 calibration_factor2（raw/g）。
 * 因子始终取正值，受力方向由 scale_direction2 单独维护，两者不再耦合。
 * 有效样本不足时拒绝更新，防止异常采样污染校准值。
 */
bool calibrateScale2(float knownWeight)
{
    if (!isScaleReady2)
    {
        return false;
    }

    const int samples = 10;
    int validCount = 0;
    float currentValue = readAverageRaw2(samples, &validCount);
    if (validCount < 5)
    {
        return false; // 有效样本不足，拒绝更新校准值
    }
    float netValue = currentValue - zero_offset2;

    // 校准因子始终为正，方向由 scale_direction2 单独处理
    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor2 = fabsf(netValue) / knownWeight;
        return true;
    }
    return false;
}

/**
 * @brief 第二个 HX711 去皮：将当前读数设为零点
 * @return true 去皮成功；false 有效样本不足，零点未更新
 *
 * 说明：
 * 1. 有效样本数不足（< TARE_MIN_VALID_SAMPLES2）时拒绝更新，防止异常采样污染零点。
 * 2. 重置 scale_direction2 和 direction_locked2，确保重新摆放后能正确识别方向。
 */
bool tareScale2()
{
    if (!isScaleReady2)
    {
        return false;
    }

    const int samples = 15;
    int validCount = 0;
    float avg = readAverageRaw2(samples, &validCount);

    if (validCount < TARE_MIN_VALID_SAMPLES2)
    {
        return false; // 有效样本不足，拒绝更新零点
    }

    zero_offset2 = avg;
    // 去皮后同时重置方向，避免旧方向导致轻载被截断为 0
    scale_direction2 = 1;
    direction_locked2 = false;
    return true;
}

/**
 * @brief 获取第二个 HX711 当前零点偏移（原始 ADC 值）
 * @return 当前零点偏移，供调用方持久化到 NVS
 */
float getZeroOffset2()
{
    return zero_offset2;
}

/**
 * @brief 直接写入第二个 HX711 的零点偏移（用于从 NVS 恢复）
 * @param offset 要恢复的零点值（原始 ADC）
 *
 * 说明：此接口仅供启动阶段从 NVS 恢复上一次有效零点使用，
 * 运行时去皮请调用 tareScale2()，不要直接调用本函数。
 */
void setZeroOffset2(float offset)
{
    zero_offset2 = offset;
    // 恢复零点时同时重置方向，允许重新识别受力方向
    scale_direction2 = 1;
    direction_locked2 = false;
}

/**
 * @brief 手动设置第二个 HX711 的校准因子
 * @param factor 校准因子（raw / g），不能为 0，始终取正值
 *
 * 适用于已通过其他方式（如 PC 软件）计算出校准因子的场景，
 * 直接写入而无需放砝码走完整的 calibrateScale2() 流程。
 */
void setCalibrationFactor2(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor2 = fabsf(factor);
    }
}

/**
 * @brief 获取第二个 HX711 当前校准因子（raw / g）
 * @return 当前校准因子，始终为正值
 */
float getCalibrationFactor2()
{
    return calibration_factor2;
}
