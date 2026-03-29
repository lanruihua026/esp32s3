#include "hx711.h"
#include "hx711BootConfig.h"

#include <math.h>

// ===== HX711 引脚定义（请按实际接线修改） =====
#define HX711_DT 1
#define HX711_SCK 2

// ===== 称重参数 =====
// calibration_factor 含义：每 1 克对应的原始 ADC 差值（raw / g），始终为正。
static float calibration_factor = 1000.0f;
// zero_offset 含义：去皮后的零点偏移（原始 ADC）
static float zero_offset = 0.0f;

// 初始化完成标志
static bool isScaleReady = false;

// 部分接线或受力方向下，施加重量可能使原始值变小（负方向）。
// scale_direction 用于统一把"加重"映射为正重量。
static int8_t scale_direction = 1;
// 一旦检测到足够明显的受力变化，就锁定方向，避免来回抖动。
static bool direction_locked = false;

// 小重量死区：将非常小的噪声视为 0g
static const float ZERO_DEADBAND_G = 2.0f;
// 方向锁定阈值：只有净原始值足够大才判断方向
static const float DIRECTION_LOCK_RAW_THRESHOLD = 30000.0f;
// 去皮时要求的最小有效样本数（共采 15 帧）
static const int TARE_MIN_VALID_SAMPLES = 8;

static int32_t readRawData();
static float readAverageRaw(int samples, int *outValidCount = nullptr);

/**
 * @brief 等待 HX711 数据就绪（DT 由高变低）
 * @param timeout_us 超时时间（微秒）
 * @return true 数据就绪；false 超时
 */
static bool waitDataReady(uint32_t timeout_us)
{
    uint32_t start = micros();
    while (digitalRead(HX711_DT) == HIGH)
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
 * @brief 初始化 HX711 模块（不自动去皮）
 * 说明：
 * 1. 设置引脚模式，确保 SCK 初始为 LOW。
 * 2. 上电后等待 1.2 秒预热，降低初始漂移影响。
 * 3. 丢弃前 8 帧数据，进一步稳定读数。
 * 4. 通过 5 帧均值探测传感器是否可用。
 * 5. 仅设置 isScaleReady 标志，不执行去皮——零点由调用方从 NVS 恢复或在确认空仓后写入。
 */
bool setupHX711()
{
    pinMode(HX711_DT, INPUT);
    pinMode(HX711_SCK, OUTPUT);
    digitalWrite(HX711_SCK, LOW);
    // 预热：策略 0 为每路等待；策略 1 由 main 在 initHx711Modules 开头统一等待
    if (HX711_BOOT_STRATEGY == 0)
    {
        delay(HX711_PER_CHANNEL_WARMUP_MS);
    }
    // 丢弃若干帧启动阶段的不稳定数据
    for (int i = 0; i < 8; ++i)
    {
        readRawData();
    }

    // 通过一次均值采样判断传感器是否可用，避免后续流程误判为"初始化成功"。
    float probeRaw = readAverageRaw(5);
    if (probeRaw == 0.0f)
    {
        isScaleReady = false;
        return false;
    }

    isScaleReady = true;
    // 不在此处去皮；零点由主流程从 NVS 恢复并按需条件去皮。
    return true;
}

/**
 * @brief 读取一帧 24 位原始值（增益 128，通道 A）
 * @return 原始值（带符号 int32），失败返回 0
 */
static int32_t readRawData()
{
    // 最长等待 100ms，避免阻塞过久
    if (!waitDataReady(100000))
    {
        return 0;
    }

    uint32_t data = 0;

    // HX711 24 位数据，高位先出
    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(HX711_SCK, HIGH);
        delayMicroseconds(1);

        if (digitalRead(HX711_DT) == HIGH)
        {
            data |= (1U << i);
        }

        digitalWrite(HX711_SCK, LOW);
        delayMicroseconds(1);
    }

    // 第 25 个时钟脉冲用于设置下一次转换的通道/增益。
    // 这里保持 A 通道、增益 128。
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);

    // 将 24 位补码扩展为 32 位有符号整数
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

/**
 * @brief 多次采样求均值，剔除无效帧
 * @param samples 采样次数（<=0 时直接返回 0）
 * @return 有效样本的平均原始 ADC 值；全部失败时返回 0
 *
 * 说明：readRawData() 失败会返回 0，这里把失败样本剔除。
 */
static float readAverageRaw(int samples, int *outValidCount)
{
    if (samples <= 0)
    {
        if (outValidCount != nullptr) *outValidCount = 0;
        return 0.0f;
    }

    int64_t sum = 0;
    int validCount = 0;

    for (int i = 0; i < samples; i++)
    {
        int32_t raw = readRawData();
        if (raw != 0)
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
 * @brief 读取当前重量（单位：克）
 * @return 当前重量（g），未就绪或重量为负时返回 0
 *
 * 说明：
 * 1. 对多次采样均值进行去皮（减去 zero_offset）。
 * 2. 首次受力超过阈值时自动锁定方向，统一让重量为正值。
 * 3. 小于死区阈值的抖动直接归零，提升静止显示稳定性。
 */
float getWeight()
{
    if (!isScaleReady)
    {
        return 0.0f;
    }

    const int samples = 5;
    int validCount = 0;
    float rawValue = readAverageRaw(samples, &validCount);
    if (validCount == 0) return 0.0f;  // 全部超时，返回0避免误算
    float netValue = rawValue - zero_offset;

    // 自动识别受力方向并锁定：
    // 如果受力后净值显著为负，后续统一乘 -1，让重量保持正向增长。
    if (!direction_locked && fabsf(netValue) > DIRECTION_LOCK_RAW_THRESHOLD)
    {
        scale_direction = (netValue >= 0.0f) ? 1 : -1;
        direction_locked = true;
    }

    float weight = (netValue * scale_direction) / calibration_factor;

    // 小抖动直接压到 0，提升静止显示稳定性
    if (fabsf(weight) < ZERO_DEADBAND_G)
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
 * @brief 获取当前载荷绝对值估计（克，不做方向裁剪）
 * @return >=0: 估计载荷；<0: 采样无效或未就绪
 */
float getLoadMagnitude()
{
    if (!isScaleReady)
    {
        return -1.0f;
    }

    int validCount = 0;
    float rawValue = readAverageRaw(5, &validCount);
    if (validCount == 0)
    {
        return -1.0f;
    }

    float netValue = rawValue - zero_offset;
    return fabsf(netValue) / calibration_factor;
}

/**
 * @brief 以已知砝码重量校准传感器
 * @param knownWeight 已知砝码的实际重量（单位：克，必须 > 0）
 * @return true 校准成功并更新因子；false 校准失败（未就绪或有效样本不足）
 *
 * 说明：
 * 校准前请确保秤盘已去皮（调用过 tareScale()）。
 * 函数通过多次采样计算新的 calibration_factor（raw/g），因子始终取正值，
 * 受力方向由 scale_direction 单独维护，两者不再耦合。
 */
bool calibrateScale(float knownWeight)
{
    if (!isScaleReady)
    {
        return false;
    }

    const int samples = 10;
    int validCount = 0;
    float currentValue = readAverageRaw(samples, &validCount);
    if (validCount < 5)
    {
        return false; // 有效样本不足，拒绝更新校准值
    }
    float netValue = currentValue - zero_offset;

    // 校准因子始终为正，方向由 scale_direction 单独处理
    if (knownWeight > 0 && netValue != 0.0f)
    {
        calibration_factor = fabsf(netValue) / knownWeight;
        return true;
    }
    return false;
}

/**
 * @brief HX711 去皮函数，将当前传感器读数设为零点
 * @return true 去皮成功；false 有效样本不足，零点未更新
 *
 * 说明：
 * 1. 读取当前原始值作为 zero_offset，后续重量计算会基于这个零点进行调整。
 * 2. 有效样本数不足（< TARE_MIN_VALID_SAMPLES）时拒绝更新，防止异常采样污染零点。
 * 3. 重置 scale_direction 和 direction_locked，确保重新摆放后能正确识别方向。
 */
bool tareScale()
{
    if (!isScaleReady)
    {
        return false;
    }

    const int samples = 15;
    int validCount = 0;
    float avg = readAverageRaw(samples, &validCount);

    if (validCount < TARE_MIN_VALID_SAMPLES)
    {
        return false; // 有效样本不足，拒绝更新零点
    }

    zero_offset = avg;
    // 去皮后同时重置方向，避免旧方向导致轻载被截断为 0
    scale_direction = 1;
    direction_locked = false;
    return true;
}

/**
 * @brief 获取当前零点偏移（原始 ADC 值）
 * @return 当前零点偏移，供调用方持久化到 NVS
 */
float getZeroOffset()
{
    return zero_offset;
}

/**
 * @brief 直接写入零点偏移（用于从 NVS 恢复）
 * @param offset 要恢复的零点值（原始 ADC）
 *
 * 说明：此接口仅供启动阶段从 NVS 恢复上一次有效零点使用，
 * 运行时去皮请调用 tareScale()，不要直接调用本函数。
 */
void setZeroOffset(float offset)
{
    zero_offset = offset;
    // 恢复零点时同时重置方向，允许重新识别受力方向
    scale_direction = 1;
    direction_locked = false;
}

/**
 * @brief 手动设置校准因子
 * @param factor 校准因子（raw / g），不能为 0
 * 说明：这个函数提供了直接设置校准因子的接口，适用于你已经通过其他方式（如 PC 软件）计算出校准因子，
 * 或者想要手动调整校准因子以微调称重结果的场景。因子始终取正值。
 */
void setCalibrationFactor(float factor)
{
    if (factor != 0.0f)
    {
        calibration_factor = fabsf(factor);
    }
}

/**
 * @brief 获取当前校准因子（raw / g）
 * @return 当前校准因子，始终为正值
 */
float getCalibrationFactor()
{
    return calibration_factor;
}
