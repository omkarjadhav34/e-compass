#include <stdint.h>
#include <stdio.h>
#include "MKL25Z4.h"
#include "externs.h"
#include "uart.h"
#include "i2c.h"
#include "Driver_I2C.h"
#include "Board_LED.h"

#define I2C_ACC_ADDR 0x19  /* I2C address of accelerometer */
#define I2C_MAG_ADDR (0x1E)  /* I2C address of magnetometer (0x1E), we have the LSM303DLHC! */

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
  
int16_t raw_x=0,raw_y=0,raw_z=0; 
uint8_t bufx_H=0, bufy_H=0, bufz_H=0,bufx_L=0, bufy_L=0, bufz_L=0;
uint8_t sr=0;

void lsm_Config()
{
		uart_String(UART0," ENabled magnetic sensor... \n");
	i2c_WriteRegister (I2C0,I2C_MAG_ADDR ,LSM303_MR_REG_M,0 );    // Mode : Continuous Conversion
	i2c_WriteRegister (I2C0,I2C_MAG_ADDR ,LSM303_CRA_REG_M,16 );  // Data output rate Configuration: 15 Hz
		i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_IRA_REG_M );    // 
		i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_IRB_REG_M );
		i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_IRC_REG_M );
		uart_String(UART0," Gain configured to +/- 1.3... \n");
		i2c_WriteRegister (I2C0,I2C_MAG_ADDR ,LSM303_CRB_REG_M ,32 );     //magnitude gain=32;
}


void spi_init()
{
	volatile unsigned long delay;
  SIM->SCGC4 |= (1 << 22);//activate SSI0
  SIM->SCGC5 |= (1 <<  11);//activate Port C
	
  PORTC->PCR[7] |= (1 << 8);//configure PTC7(DC) as GPIO(ALT1) 
  PORTC->PCR[8] |= (1 << 8);//configure PTC8(RST) as GPIO(ALT1)
  GPIOC->PDDR |= (1 << 7)|(1 << 8);//set PTC7(DC),PTC8(RST) as output pins 
  
  PORTC->PCR[4] |= (0x2 << 8);//configure PTC4 as SPI SS(ALT2) 
  PORTC->PCR[5] |= (0x2 << 8);//configure PTC5 as SPI Clock(ALT2)
  PORTC->PCR[6] |= (0x2 << 8);//configure PTC6 as SPI MOSI(ALT2)
  
  SPI0->C1 &= ~(1 << 6);//disable SSI 
  SPI0->C1 |= (1 << 4);//master mode
  SPI0->C1 &= ~(1 << 0);//SPI serial data transfers start with MSB
  SPI0->C1 &= ~(1 << 3);//CPOL=0
  SPI0->C1 &= ~(1 << 2);//CPHA=0
  /*
  SSOE, MODFEN=1 --> SS pin (of Master) used as Slave Select Output
  */
  SPI0->C1 |= (1 << 1);//SSOE=1
  SPI0->C2 |= (1 << 4);//MODFEN=1
  /*
  BRD = (SPPR + 1) * 2^(SPR + 1)
  SPPR=0
  SPR=2
  BRD = 1 * 2^3 = 8
  Baud Rate = Bus Clock Freq./BRD = 24 MHz (default)/8 = 3 MHz
  */
  SPI0->BR &= ~(0x7 << 4);//SPPR=0
  SPI0->BR |= (0x2 << 0);//SPR=2
  SPI0->C1 |= (1 << 6);//enable SSI

  GPIOC->PDOR &= ~(1 << 8);// hold RST(PTC8) pin LOW
  for(delay=0; delay<5; delay++);// delay minimum 100 ns
  GPIOC->PDOR |= (1 << 8);// set RST(PTC8) pin HIGH
}
int Main (void) 
	{
		SystemCoreClock =24000000;                                // Set Clock speed : 24MHz
		SystemCoreClockUpdate();			                            // Update clockspeed 
		uart_Init ( (UART_MemMapPtr)UART0,ALT0,UART_BAUD_1 );     // Uart initilization
		i2c_Init(I2C0, ALT1, MULT0, 0x1F);                        //I2C Initialized
		uart_String(UART0, "UART, I2C Initialized\n");
		lsm_Config();
		//Nokia5110_Init();
	
		while(1)
		{
			sr=i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_SR_REG_M);
			
			bufx_H= i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_OUT_X_H_M  );
			bufx_L= i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_OUT_X_L_M  );
			bufy_H= i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_OUT_Y_H_M  );
			bufy_L= i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_OUT_Y_L_M  );
			raw_y =(int16_t )(((uint8_t)bufy_H <<8)|(uint8_t)bufy_L )/11;
			raw_x =(int16_t )(((uint8_t)bufx_H <<8)|(uint8_t)bufx_L )/11;
			
			sr=i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_SR_REG_M);
			uart_String(UART0,"X-axis \n");
			uart_Put(UART0,(raw_x/10)+48);  //converting to ASCII characters
			uart_Put(UART0,(raw_x%10)+48);
			bufx_H=0;
			bufx_L=0;
			uart_String(UART0,"Y-axis \n");
			uart_Put(UART0,(raw_y/10 )+48); //converting to ASCII characters
			uart_Put(UART0,(raw_y%10)+48);
			bufy_H=0;
			bufy_L=0;
			
			i2c_WriteRegister (I2C0,I2C_MAG_ADDR ,LSM303_SR_REG_M,(0<<1|0));
			delay(2000);
			
			sr=i2c_ReadRegister (I2C0,I2C_MAG_ADDR ,LSM303_SR_REG_M);
			i2c_WriteRegister (I2C0,I2C_MAG_ADDR ,LSM303_MR_REG_M,0 );

}
}
	
