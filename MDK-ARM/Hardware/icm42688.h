#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"

/* 保留参考工程接线。SPI 引脚配置在 main.c 的 MX_SPI2_Init 和 MSP 中；
 * 更换引脚时同时修改 .ioc。CS 使用 main.h 中的 IMU_CS 定义。 */
typedef struct
{
    float accel[3];                 /* mg，参考工程量程为 +/-4g */
    float gyro[3];                  /* deg/s，量程为 +/-1000deg/s */
} ICM42688_Sample;

uint8_t ICM42688_Init(void);
/* 1=有效新样本，0=无可用新样本（未就绪或异常帧），-1=通信/身份检查失败。
 * 返回 0/-1 时不修改 *sample。仅主循环调用，传入有效的输出地址。 */
int8_t ICM42688_Read(ICM42688_Sample *sample);

#endif
