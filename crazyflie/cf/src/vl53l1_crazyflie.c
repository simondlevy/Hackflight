#include <st/vl53l1_api.h>

#include <hal/i2cdev.h>

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t usec)
{
    void delayMicroseconds(const uint32_t usec);

    delayMicroseconds(usec);

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, 
        uint8_t * pdata, uint32_t count)
{
    return i2cdevWriteReg16((I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
        VL53L1_ERROR_NONE : 
        VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, 
        uint8_t * pdata, uint32_t   count)
{
    return i2cdevReadReg16((I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
        VL53L1_ERROR_NONE : 
        VL53L1_ERROR_CONTROL_INTERFACE;
}
