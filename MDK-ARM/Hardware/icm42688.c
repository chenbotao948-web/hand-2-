#include "icm42688.h"

/* 对照用户 STM32F4_ICM42688_HAL/HARDWARE/ICM42688（Baxiange）适配 F103 HAL。
 * 保留 SPI2 mode 3、100Hz、+/-4g、+/-1000deg/s；按示例读取连续六轴数据。 */
extern SPI_HandleTypeDef hspi2;

#define ICM_DEVICE_CONFIG    0x11U
#define ICM_ACCEL_DATA       0x1FU
#define ICM_INT_STATUS       0x2DU
#define ICM_PWR_MGMT0        0x4EU
#define ICM_GYRO_CONFIG0     0x4FU
#define ICM_ACCEL_CONFIG0    0x50U
#define ICM_WHO_AM_I         0x75U
#define ICM_BANK_SEL         0x76U
#define ICM_DEVICE_ID        0x47U
#define ICM_SPI_TIMEOUT_MS   3U

static uint8_t ICM_Transfer(uint8_t reg, uint8_t *data, uint8_t length, uint8_t read)
{
    uint8_t tx[13] = {0};
    uint8_t rx[13];
    uint8_t i;
    HAL_StatusTypeDef status;

    if ((length == 0U) || (length > 12U)) return 0U;
    tx[0] = read ? (uint8_t)(reg | 0x80U) : reg;
    if (!read)
    {
        for (i = 0U; i < length; i++) tx[i + 1U] = data[i];
    }
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, length + 1U, ICM_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    if (status != HAL_OK) return 0U;
    if (read)
    {
        for (i = 0U; i < length; i++) data[i] = rx[i + 1U];
    }
    return 1U;
}

static uint8_t ICM_Write(uint8_t reg, uint8_t value)
{
    return ICM_Transfer(reg, &value, 1U, 0U);
}

uint8_t ICM42688_Init(void)
{
    uint8_t id;
    if (!ICM_Write(ICM_BANK_SEL, 0U) ||
        !ICM_Transfer(ICM_WHO_AM_I, &id, 1U, 1U) || id != ICM_DEVICE_ID)
        return 0U;
    if (!ICM_Write(ICM_DEVICE_CONFIG, 0x01U)) return 0U;
    HAL_Delay(100U); /* 只在启动/故障恢复时等待复位；正常采样不延时。 */
    if (!ICM_Write(ICM_BANK_SEL, 0U) ||
        !ICM_Write(ICM_ACCEL_CONFIG0, 0x48U) ||
        !ICM_Write(ICM_GYRO_CONFIG0, 0x28U) ||
        !ICM_Write(ICM_PWR_MGMT0, 0x0FU)) return 0U;
    HAL_Delay(100U); /* 等待陀螺仪启动稳定，然后再进行静止校准。 */
    if (!ICM_Transfer(ICM_WHO_AM_I, &id, 1U, 1U) || id != ICM_DEVICE_ID)
        return 0U;
    return 1U;
}

int8_t ICM42688_Read(ICM42688_Sample *sample)
{
    uint8_t id, status, raw[12], i;
    int16_t accel[3], gyro[3];
    /* SPI 在拔线时也可能返回 HAL_OK，因此每次核对芯片身份。 */
    if (!ICM_Transfer(ICM_WHO_AM_I, &id, 1U, 1U) || id != ICM_DEVICE_ID ||
        !ICM_Transfer(ICM_INT_STATUS, &status, 1U, 1U)) return -1;
    if ((status & 0x08U) == 0U) return 0;
    if (!ICM_Transfer(ICM_ACCEL_DATA, raw, 12U, 1U)) return -1;
    for (i = 0U; i < 3U; i++)
    {
        accel[i] = (int16_t)(((uint16_t)raw[2U*i] << 8) | raw[1U + 2U*i]);
        gyro[i] = (int16_t)(((uint16_t)raw[6U + 2U*i] << 8) | raw[7U + 2U*i]);
    }
    /* 保留 HAL 示例的加速度全 -1 拒绝规则，将 -32768 检查覆盖全部六轴。
     * 先检查整帧，再写输出；异常帧不参与校准/积分，也不刷新有效样本时间。 */
    if (accel[0] == -1 && accel[1] == -1 && accel[2] == -1) return 0;
    for (i = 0U; i < 3U; i++)
    {
        if (accel[i] == -32768 || gyro[i] == -32768) return 0;
    }
    for (i = 0U; i < 3U; i++)
    {
        sample->accel[i] = (float)accel[i] * (4000.0f / 32768.0f);
        sample->gyro[i] = (float)gyro[i] * (1000.0f / 32768.0f);
    }
    return 1;
}
