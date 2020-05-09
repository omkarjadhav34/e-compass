//#include "LSM303.h"
#include "Driver_I2C.h"

#define I2C_ACC_ADDR 0x19  /* I2C address of accelerometer */
#define I2C_MAG_ADDR (0x3C>>1)  /* I2C address of magnetometer (0x1E), we have the LSM303DLHC! */

// register addresses

#define LSM303_CTRL_REG1_A 0x20
#define LSM303_CTRL_REG2_A 0x21
#define LSM303_CTRL_REG3_A 0x22
#define LSM303_CTRL_REG4_A 0x23
#define LSM303_CTRL_REG5_A 0x24
#define LSM303_CTRL_REG6_A 0x25 // DLHC only
#define LSM303_HP_FILTER_RESET_A 0x25 // DLH, DLM only
#define LSM303_REFERENCE_A 0x26
#define LSM303_STATUS_REG_A 0x27

#define LSM303_OUT_X_L_A 0x28
#define LSM303_OUT_X_H_A 0x29
#define LSM303_OUT_Y_L_A 0x2A
#define LSM303_OUT_Y_H_A 0x2B
#define LSM303_OUT_Z_L_A 0x2C
#define LSM303_OUT_Z_H_A 0x2D

#define LSM303_FIFO_CTRL_REG_A 0x2E // DLHC only
#define LSM303_FIFO_SRC_REG_A 0x2F // DLHC only

#define LSM303_INT1_CFG_A 0x30
#define LSM303_INT1_SRC_A 0x31
#define LSM303_INT1_THS_A 0x32
#define LSM303_INT1_DURATION_A 0x33
#define LSM303_INT2_CFG_A 0x34
#define LSM303_INT2_SRC_A 0x35
#define LSM303_INT2_THS_A 0x36
#define LSM303_INT2_DURATION_A 0x37

#define LSM303_CLICK_CFG_A 0x38 // DLHC only
#define LSM303_CLICK_SRC_A 0x39 // DLHC only
#define LSM303_CLICK_THS_A 0x3A // DLHC only
#define LSM303_TIME_LIMIT_A 0x3B // DLHC only
#define LSM303_TIME_LATENCY_A 0x3C // DLHC only
#define LSM303_TIME_WINDOW_A 0x3D // DLHC only

#define LSM303_CRA_REG_M 0x00
#define LSM303_CRB_REG_M 0x01
#define LSM303_MR_REG_M 0x02

#define LSM303_OUT_X_H_M 0x03
#define LSM303_OUT_X_L_M 0x04
#define LSM303_OUT_Y_H_M 0x07
#define LSM303_OUT_Y_L_M 0x08
#define LSM303_OUT_Z_H_M 0x05
#define LSM303_OUT_Z_L_M 0x06

#define LSM303_SR_REG_M 0x09
#define LSM303_IRA_REG_M 0x0A
#define LSM303_IRB_REG_M 0x0B
#define LSM303_IRC_REG_M 0x0C

#define LSM303_WHO_AM_I_M 0x0F // DLM only

#define LSM303_TEMP_OUT_H_M 0x31 // DLHC only
#define LSM303_TEMP_OUT_L_M 0x32 // DLHC only

#define LSM303DLH_OUT_Y_H_M 0x05
#define LSM303DLH_OUT_Y_L_M 0x06
#define LSM303DLH_OUT_Z_H_M 0x07
#define LSM303DLH_OUT_Z_L_M 0x08

#define LSM303DLM_OUT_Z_H_M 0x05
#define LSM303DLM_OUT_Z_L_M 0x06
#define LSM303DLM_OUT_Y_H_M 0x07
#define LSM303DLM_OUT_Y_L_M 0x08

#define LSM303DLHC_OUT_Z_H_M 0x05
#define LSM303DLHC_OUT_Z_L_M 0x06
#define LSM303DLHC_OUT_Y_H_M 0x07
#define LSM303DLHC_OUT_Y_L_M 0x08

void LSM_MagEnable(void) {
  /* enable magnetometer in continuous mode */
  uint8_t addr = LSM303_MR_REG_M;
  uint8_t data = 0x0;
  
  (void)I2C_WriteAddress(I2C_MAG_ADDR, &addr, sizeof(addr), &data, sizeof(data));
}





void LSM_ReadMag(uint16_t *x, uint16_t *y, uint16_t *z) {
  uint8_t addr = LSM303_OUT_X_H_M;
  uint8_t buf[6]; /* x, y and z */

  (void)I2C2_ReadAddress(I2C_MAG_ADDR, &addr, sizeof(addr), buf, sizeof(buf));
  *x = (buf[0]<<8)|buf[1];
  *y = (buf[4]<<8)|buf[5];
  *z = (buf[2]<<8)|buf[3];
}


void LSM_Init(void) {
  LSM_MagEnable();
 // LSM_MagTempEnable();
}