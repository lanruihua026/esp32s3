#ifndef HX711_2_H
#define HX711_2_H

#include <Arduino.h>

/**
 * @brief 初始化第二个 HX711 称重模块（DT=GPIO11, SCK=GPIO12，不自动去皮）
 *
 * 包含引脚初始化、预热、丢弃前几帧不稳定数据与传感器可用性探测。
 * 零点偏移由调用方从 NVS 恢复或在确认空仓后写入，本函数不执行去皮。
 * @return true 初始化成功；false 初始化失败（数据未就绪）
 */
bool setupHX711_2();

/**
 * @brief 获取第二个 HX711 当前重量（克）
 * @return 当前重量（g）
 */
float getWeight2();

/**
 * @brief 获取第二个 HX711 当前载荷绝对值估计（克，不做方向裁剪）
 * @return >=0: 估计载荷；<0: 采样无效或未就绪
 */
float getLoadMagnitude2();

/**
 * @brief 使用已知砝码对第二个 HX711 执行校准
 * @param knownWeight 已知重量（g）
 * @return true 校准成功并更新因子；false 校准失败（未就绪或有效样本不足）
 */
bool calibrateScale2(float knownWeight);

/**
 * @brief 第二个 HX711 去皮（将当前读数设为零点）
 * @return true 去皮成功；false 有效样本不足，零点未更新
 */
bool tareScale2();

/**
 * @brief 手动设置第二个 HX711 的校准因子
 * @param factor 校准因子（raw / g），始终取正值
 */
void setCalibrationFactor2(float factor);

/**
 * @brief 获取第二个 HX711 当前校准因子（raw / g）
 * @return 当前校准因子，始终为正值
 */
float getCalibrationFactor2();

/**
 * @brief 获取第二个 HX711 当前零点偏移（原始 ADC 值）
 * @return 当前零点偏移，供调用方持久化到 NVS
 */
float getZeroOffset2();

/**
 * @brief 直接写入第二个 HX711 的零点偏移（用于从 NVS 恢复）
 * @param offset 要恢复的零点值（原始 ADC）
 */
void setZeroOffset2(float offset);


#endif // HX711_2_H
