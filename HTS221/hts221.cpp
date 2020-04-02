/** 
 *  hts221.cpp  
 *  This is a C++ class implementation to read the HTS221  humid
 * ( and temperature sensor) 
 * substantial part of the code is copied from the AST Robotics Team 
 * see their copy right below. 
 * This program will be used to monitor the humitidy in the SOLID detector. 
 * W. Beaumont
 * (C) University Antwerpen
 * version history  
 * version 0.1  : can read the ID 
 * version 0.6x : worked on MBED  
 * version 0.70 : debug version , work only partially on linux 
 * version 0.78 : work on linux , checking more 
 **/
/**
 ******************************************************************************
 * file    hts221.c
 * author  AST Robotics Team
 * version V0.0.1
 * date    08-April-2014
 * brief   This file provides a set of functions needed to manage the hts221.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "hts221.h"
#include <math.h>
#define VERSION_HTS221_SRC "0.78"

//#define DEBUGCODE 
// only works with mbed 

#ifdef DEBUGCODE 

#ifdef __MBED__ 
#include "mbed.h"
#else 
#include <stdio.h>
int LED1=3;
class DigitalOut {
	int led;
	public:
	DigitalOut( int lednr) {
       led=0;
	}
	
	DigitalOut operator !( ){ if ( led )  led=0; else led=1; return *this  ;}

};
#endif // __MBED__
#endif // DEBUGCODE


	

#define HTS221_ADDRESS                              0xBE

/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/

/**
 * @brief Device identification register.
 * \code
   * Read
 * Default value: 0xBC
 * 7:0 This read-only register contains the device identifier that, for HTS221, is set to BCh.
 * \endcode
*/
#define HTS221_WHO_AM_I_ADDR                        0x0F

   /**
    * @brief Humidity resolution Register
    * \code
    * Read/write
    * Default value: 0x1B
    * 7:6 RFU
    * 5:3 AVGT2-AVGT0: Temperature internal average.
    *     AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
    *   ------------------------------------------------------
    *      0    |  0    |  0    |     2
    *      0    |  0    |  1    |     4
    *      0    |  1    |  0    |     8
    *      0    |  1    |  1    |     16
    *      1    |  0    |  0    |     32
    *      1    |  0    |  1    |     64
    *      1    |  1    |  0    |     128
    *      1    |  1    |  1    |     256
    *
    * 2:0 AVGH2-AVGH0: Humidity internal average.
    *     AVGH2 | AVGH1 | AVGH0 | Nr. Internal Average
    *   ------------------------------------------------------
    *      0    |  0    |  0    |     4
    *      0    |  0    |  1    |     8
    *      0    |  1    |  0    |     16
    *      0    |  1    |  1    |     32
    *      1    |  0    |  0    |     64
    *      1    |  0    |  1    |     128
    *      1    |  1    |  0    |     256
    *      1    |  1    |  1    |     512
    *
    * \endcode
    */
#define HTS221_RES_CONF_ADDR                        0x10

    /**
    * @brief INFO Register  (LSB data)
    * \code
    * Read/write
    * Default value: 0x00
    * 7:0 INFO7-INFO0: Lower part of the INFO reference
    *                  used for traceability of the sample.
    * \endcode
    */
#define HTS221_INFO_L_ADDR                          0x1E

    /**
    * @brief INFO & Calibration Version Register  (LSB data)
    * \code
    * Read/write
    * Default value: 0x00
    * 7:6 CALVER1:CALVER0
    * 5:0 INFO13-INFO8: Higher part of the INFO reference
    *                  used for traceability of the sample.
    * \endcode
    */
#define HTS221_INFO_H_ADDR                          0x1F

    /**
    * @brief Humidity sensor control register 1
    * \code
    * Read/write
    * Default value: 0x00
    * 7    PD: power down control. 0 - disable; 1 - enable
    * 6:3  RFU
    * 2    BDU: block data update. 0 - disable; 1 - enable
    * 1:0  RFU
    * \endcode
    */

#define HTS221_CTRL_REG1_ADDR                       0x20


    /**
    * @brief Humidity sensor control register 2
    * \code
    * Read/write
    * Default value: 0x00
    * 7    BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content
    * 6:3  Reserved.
    * 2    Reserved.
    * 1    Reserved.
    * 0    ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
    * \endcode
    */
#define HTS221_CTRL_REG2_ADDR                       0x21

    /**
    * @brief Humidity sensor control register 3
    * \code
    * Read/write
    * Default value: 0x00  enable de DRDY  pin 
    * \endcode
    */

#define HTS221_CTRL_REG3_ADDR                       0x22

    /**
    * @brief  Status Register
    * \code
    * Read
    * Default value: 0x00
    * 7:2  RFU
    * 1    H_DA: Humidity data available. 0: new data for Humidity is not yet available; 1: new data for Humidity is available.
    * 0    T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
    * \endcode
    */
#define HTS221_STATUS_REG_ADDR                      0x27


    /**
    * @brief  Humidity data (LSB).
    * \code
    * Read
    * Default value: 0x00.
    * POUT7 - POUT0: Humidity data LSB (2's complement) => signed 16 bits
    * RAW Humidity output data: Hout(%)=(HUMIDITY_OUT_H & HUMIDITY_OUT_L).
    * \endcode
    */
#define HTS221_HUMIDITY_OUT_L_ADDR                  0x28


    /**
    * @brief  Humidity data (MSB).
    * \code
    * Read
    * Default value: 0x00.
    * POUT7 - POUT0: Humidity data LSB (2's complement) => signed 16 bits
    * RAW Humidity output data: Hout(%)=(HUMIDITY_OUT_H & HUMIDITY_OUT_L).
    * \endcode
    */
#define HTS221_HUMIDITY_OUT_H_ADDR                  0x29


    /**
    * @brief  Temperature data (LSB).
    * \code
    * Read
    * Default value: 0x00.
    * TOUT7 - TOUT0: temperature data LSB (2's complement) => signed 16 bits
    * RAW Temperature output data: Tout (LSB)=(TEMP_OUT_H & TEMP_OUT_L).
    * \endcode
    */
#define HTS221_TEMP_OUT_L_ADDR                      0x2A


    /**
    * @brief  Temperature data (MSB).
    * \code
    * Read
    * Default value: 0x00.
    * TOUT15 - TOUT8: temperature data MSB (2's complement) => signed 16 bits
    * RAW Temperature output data: Tout (LSB)=(TEMP_OUT_H & TEMP_OUT_L).
    * \endcode
    */
#define HTS221_TEMP_OUT_H_ADDR                      0x2B


    /*
    *@brief Humidity 0 Register in %RH with sensitivity=2
    *\code
    * Read
    * Value: (Unsigned 8 Bit)/2
    *\endcode
    */
#define HTS221_H0_RH_X2_ADDR                        0x30


    /*
    *@brief Humidity 1 Register in %RH with sensitivity=2
    *\code
    * Read
    * Value: (Unsigned 8 Bit)/2
    *\endcode
    */
#define HTS221_H1_RH_X2_ADDR                        0x31


    /*
    *@brief Temperature 0 Register in deg with sensitivity=8
    *\code
    * Read
    * Value: (Unsigned 16 Bit)/2
    *\endcode
    */
#define HTS221_T0_degC_X8_ADDR                      0x32


    /*
    *@brief Temperature 1 Register in deg with sensitivity=8
    *\code
    * Read
    * Value: (Unsigned 16 Bit)/2
    *\endcode
    */
#define HTS221_T1_degC_X8_ADDR                      0x33


    /*
    *@brief Temperature 1/0 MSB Register in deg with sensitivity=8
    *\code
    * Read
    * Value: (Unsigned 16 Bit)/2
    * 3:2  T1(9):T1(8) MSB T1_degC_X8 bits
    * 1:0  T0(9):T0(8) MSB T0_degC_X8 bits
    *\endcode
    */
#define HTS221_T1_T0_MSB_X8_ADDR                    0x35


    /*
    *@brief Humidity LOW CALIBRATION Register
    *\code
    * Read
    * Default value: 0x00.
    * H0_T0_TOUT7 - H0_T0_TOUT0: HUMIDITY data lSB (2's complement) => signed 16 bits
    *\endcode
    */
#define HTS221_H0_T0_OUT_L_ADDR                     0x36


    /*
    *@brief Humidity LOW CALIBRATION Register
    *\code
    * Read
    * Default value: 0x00.
    * H0_T0_TOUT15 - H0_T0_TOUT8: HUMIDITY data mSB (2's complement) => signed 16 bits
    *\endcode
    */
#define HTS221_H0_T0_OUT_H_ADDR                       0x37


    /*
    *@brief Humidity HIGH CALIBRATION Register
    *\code
    * Read
    * Default value: 0x00.
    * H1_T0_TOUT7 - H1_T0_TOUT0: HUMIDITY data lSB (2's complement) => signed 16 bits
    *\endcode
    */
#define HTS221_H1_T0_OUT_L_ADDR                       0x3A


    /*
    *@brief Humidity HIGH CALIBRATION Register
    *\code
    * Read
    * Default value: 0x00.
    * H1_T0_TOUT15 - H1_T0_TOUT8: HUMIDITY data mSB (2's complement) => signed 16 bits
    *\endcode
    */
#define HTS221_H1_T0_OUT_H_ADDR                       0x3B


    /**
    * @brief  Low Calibration Temperature Register (LSB).
    * \code
    * Read
    * Default value: 0x00.
    * T0_OUT7 - T0_OUT0: temperature data LSB (2's complement) => signed 16 bits
    *  RAW LOW Calibration data: T0_OUT (LSB)=(T0_OUT_H & T0_OUT_L).
    * \endcode
    */
#define HTS221_T0_OUT_L_ADDR                        0x3C


    /**
    * @brief  Low Calibration Temperature Register (MSB)
    * \code
    * Read
    * Default value: 0x00.
    * T0_OUT15 - T0_OUT8: temperature data MSB (2's complement) => signed 16 bits
    * RAW LOW Calibration data: T0_OUT (LSB)=(T0_OUT_H & T0_OUT_L).
    * \endcode
    */
#define HTS221_T0_OUT_H_ADDR                        0x3D


    /**
    * @brief  Low Calibration Temperature Register (LSB).
    * \code
    * Read
    * Default value: 0x00.
    * T1_OUT7 - T1_OUT0: temperature data LSB (2's complement) => signed 16 bits
    *  RAW LOW Calibration data: T1_OUT (LSB)=(T1_OUT_H & T1_OUT_L).
    * \endcode
    */
#define HTS221_T1_OUT_L_ADDR                        0x3E


    /**
    * @brief  Low Calibration Temperature Register (MSB)
    * \code
    * Read
    * Default value: 0x00.
    * T1_OUT15 - T1_OUT8: temperature data MSB (2's complement) => signed 16 bits
    * RAW LOW Calibration data: T1_OUT (LSB)=(T1_OUT_H & T1_OUT_L).
    * \endcode
    */
#define HTS221_T1_OUT_H_ADDR                        0x3F


/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/


/**
 * @brief Device Identifier. Default value of the WHO_AM_I register.
 */
#define I_AM_HTS221                         ((uint8_t)0xBC)


/** @defgroup HTS221 Power Mode selection - CTRL_REG1
  * @{
  */
#define HTS221_MODE_POWERDOWN               ((uint8_t)0x00)
#define HTS221_MODE_ACTIVE                  ((uint8_t)0x80)

#define HTS221_MODE_MASK                    ((uint8_t)0x80)
/**
  * @}
  */ 


/** @defgroup HTS221 Block Data Update Mode selection - CTRL_REG1
  * @{
  */
#define HTS221_BDU_CONTINUOUS               ((uint8_t)0x00)
#define HTS221_BDU_NOT_UNTIL_READING        ((uint8_t)0x04)

#define HTS221_BDU_MASK                     ((uint8_t)0x04)
/**
  * @}
  */

/** @defgroup HTS221 Output Data Rate selection - CTRL_REG1
 * @{
 */
#define HTS221_ODR_ONE_SHOT             ((uint8_t)0x00) /*!< Output Data Rate: H - one shot, T - one shot */
#define HTS221_ODR_1Hz                  ((uint8_t)0x01) /*!< Output Data Rate: H - 1Hz, T - 1Hz */
#define HTS221_ODR_7Hz                  ((uint8_t)0x02) /*!< Output Data Rate: H - 7Hz, T - 7Hz */
#define HTS221_ODR_12_5Hz               ((uint8_t)0x03) /*!< Output Data Rate: H - 12.5Hz, T - 12.5Hz */

#define HTS221_ODR_MASK                 ((uint8_t)0x03)
/**
* @}
*/


/** @defgroup HTS221 Boot Mode selection - CTRL_REG2
  * @{
  */
#define HTS221_BOOT_NORMALMODE              ((uint8_t)0x00)
#define HTS221_BOOT_REBOOTMEMORY            ((uint8_t)0x80)

#define HTS221_BOOT_MASK                    ((uint8_t)0x80)
/**
  * @}
  */  


/** @defgroup HTS221 One Shot selection - CTRL_REG2
 * @{
 */
#define HTS221_ONE_SHOT_START               ((uint8_t)0x01)

#define HTS221_ONE_SHOT_MASK                ((uint8_t)0x01)
/**
 * @}
 */


/** @defgroup HTS221 Boot Mode selection - CTRL_REG2
  * @{
  */
#define HTS221_BOOT_NORMALMODE              ((uint8_t)0x00)
#define HTS221_BOOT_REBOOTMEMORY            ((uint8_t)0x80)

#define HTS221_BOOT_MASK                    ((uint8_t)0x80)
/**
  * @}
  */


/** @defgroup HTS221 PushPull_OpenDrain selection - CTRL_REG3
  * @{
  */
#define HTS221_PP_OD_PUSH_PULL              ((uint8_t)0x00)
#define HTS221_PP_OD_OPEN_DRAIN             ((uint8_t)0x40)

#define HTS221_PP_OD_MASK                   ((uint8_t)0x40)
/**
  * @}
  */


/** @defgroup HTS221 Data ready selection - CTRL_REG3
  * @{
  */
 // doubt if these values are correct but not used up to now . 
#define HTS221_DRDY_DISABLE                 ((uint8_t)0x00)
#define HTS221_DRDY_AVAILABLE               ((uint8_t)0x40)

#define HTS221_DRDY_MASK                    ((uint8_t)0x40)
/**
  * @}
  */


/** @defgroup HTS221 Humidity resolution selection - RES_CONF
  * @{
  */
#define HTS221_H_RES_AVG_4                  ((uint8_t)0x00)
#define HTS221_H_RES_AVG_8                  ((uint8_t)0x01)
#define HTS221_H_RES_AVG_16                 ((uint8_t)0x02)
#define HTS221_H_RES_AVG_32                 ((uint8_t)0x03)
#define HTS221_H_RES_AVG_64                 ((uint8_t)0x04)
#define HTS221_H_RES_AVG_128                ((uint8_t)0x05)

#define HTS221_H_RES_MASK                   ((uint8_t)0x07)
/**
  * @}
  */


/** @defgroup HTS221 Temperature resolution - RES_CONF
  * @{
  */
#define HTS221_T_RES_AVG_2                  ((uint8_t)0x00)
#define HTS221_T_RES_AVG_4                  ((uint8_t)0x08)
#define HTS221_T_RES_AVG_8                  ((uint8_t)0x10)
#define HTS221_T_RES_AVG_16                 ((uint8_t)0x18)
#define HTS221_T_RES_AVG_32                 ((uint8_t)0x20)
#define HTS221_T_RES_AVG_64                 ((uint8_t)0x28)

#define HTS221_T_RES_MASK                   ((uint8_t)0x38)
/**
  * @}
  */


/** @defgroup HTS221 Temperature Humidity data available - STATUS_REG
  * @{
  */
#define HTS221_H_DATA_AVAILABLE_MASK        ((uint8_t)0x02)
#define HTS221_T_DATA_AVAILABLE_MASK        ((uint8_t)0x01)
/**
  * @}
  */



/* Data resolution */
#define HUM_DECIMAL_DIGITS                  (2)
#define TEMP_DECIMAL_DIGITS                 (2)

 


HTS221::HTS221(I2CInterface* i2cinterface, bool init , bool OneShot ):getVersion( VERSION_HTS221_HDR,VERSION_HTS221_SRC, __TIME__, __DATE__),
    _i2c(i2cinterface) {
 #ifdef DEBUGCODE 
        printf("MBED debug active \n\r");
#endif         
    if( init){ 
		if( OneShot ) OutputDataRate=HTS221_ODR_ONE_SHOT;
		else OutputDataRate=HTS221_ODR_7Hz;
        Init();//power on, get cal values  ,  set readout rate  
    }
};





/**
 * @brief  HTS221 Calibration procedure.
 * @param  None
 * @retval None
 */
void HTS221::Calibration(void)
{
    /* Temperature Calibration */
    /* Temperature in degree for calibration ( "/8" to obtain float) */
    uint16_t T0_degC_x8_L, T0_degC_x8_H, T1_degC_x8_L, T1_degC_x8_H;
    uint8_t H0_rh_x2, H1_rh_x2;
    uint8_t tempReg[2] = {0,0};

    HUM_TEMP_IO_Read(tempReg,  HTS221_T0_degC_X8_ADDR, 1);
    T0_degC_x8_L = (uint16_t)tempReg[0];

    HUM_TEMP_IO_Read(tempReg,  HTS221_T1_T0_MSB_X8_ADDR, 1);
    T0_degC_x8_H = (uint16_t) (tempReg[0] & 0x03);

    T0_degC = ((float)((T0_degC_x8_H<<8) | (T0_degC_x8_L)))/8;

    HUM_TEMP_IO_Read(tempReg,  HTS221_T1_degC_X8_ADDR, 1);
    T1_degC_x8_L = (uint16_t)tempReg[0];

    HUM_TEMP_IO_Read(tempReg,  HTS221_T1_T0_MSB_X8_ADDR, 1);
    T1_degC_x8_H = (uint16_t) (tempReg[0] & 0x0C);
    T1_degC_x8_H = T1_degC_x8_H >> 2;

    T1_degC = ((float)((T1_degC_x8_H<<8) | (T1_degC_x8_L)))/8;

    HUM_TEMP_IO_Read(tempReg,  HTS221_T0_OUT_L_ADDR + 0x80, 2);
    T0_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    HUM_TEMP_IO_Read(tempReg,  HTS221_T1_OUT_L_ADDR + 0x80, 2);
    T1_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    /* Humidity Calibration */
    /* Humidity in degree for calibration ( "/2" to obtain float) */

    HUM_TEMP_IO_Read(&H0_rh_x2,  HTS221_H0_RH_X2_ADDR, 1);

    HUM_TEMP_IO_Read(&H1_rh_x2,  HTS221_H1_RH_X2_ADDR, 1);

    HUM_TEMP_IO_Read(&tempReg[0],  HTS221_H0_T0_OUT_L_ADDR + 0x80, 2);
    H0_T0_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

#ifdef DEBUGCODE 
        printf("H0_rh_x2 %02X , %d ;H1_rh_x2 %02X , %d ;",  H0_rh_x2,H0_rh_x2,H1_rh_x2,H1_rh_x2);
        printf("MSB H0_TO_out %02X , %d ;LSB H0_TO_out %02X , %d \n\r",  tempReg[1],tempReg[1],tempReg[0],tempReg[0]);
#endif    


    HUM_TEMP_IO_Read(&tempReg[0],  HTS221_H1_T0_OUT_L_ADDR  + 0x80, 2);
    H1_T0_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    H0_rh = ((float)H0_rh_x2)/2;
    H1_rh = ((float)H1_rh_x2)/2;
}


/**
 * @brief  Set HTS221 Initialization.
 * @param  InitStruct: it contains the configuration setting for the HTS221.
 * @retval None
 */
void HTS221::Init(void) {  
     uint8_t tmp[2] = {0,0x00 };
    

    Power_On();
    Calibration();

    HUM_TEMP_IO_Read(&tmp[1],  HTS221_CTRL_REG1_ADDR, 1);
	
    /* Output Data Rate selection */
    tmp[1] &= ~(HTS221_ODR_MASK);
    tmp[1] |= OutputDataRate;
	tmp[0] = HTS221_CTRL_REG1_ADDR;
    HUM_TEMP_IO_Write(tmp,  2);

}

/**
 * @brief  Read ID address of HTS221
 * @param  Device ID address
 * @retval ID name
 */
uint8_t HTS221::ReadID(void)
{
    uint8_t tmp;

    /* Read WHO I AM register */
    HUM_TEMP_IO_Read(&tmp,  HTS221_WHO_AM_I_ADDR, 1);
	

    /* Return the ID */
    return (uint8_t)tmp;
}

/**
 * @brief  Reboot memory content of HTS221
 * @param  None
 * @retval None
 */
void HTS221::RebootCmd(void)
{
    uint8_t tmpreg[2];

    /* Read CTRL_REG2 register */
    HUM_TEMP_IO_Read(&tmpreg[1],  HTS221_CTRL_REG2_ADDR, 1);

    /* Enable or Disable the reboot memory */
    tmpreg[1] |= HTS221_BOOT_REBOOTMEMORY;
	tmpreg[0]=HTS221_CTRL_REG2_ADDR;
    /* Write value to MEMS CTRL_REG2 regsister */
    HUM_TEMP_IO_Write(tmpreg,  2);
}



/**
 * @brief  Read HTS221 output register, and calculate the humidity.
 * @param  pfData : Data out pointer
 * @retval 1 in case of I2C error , 0x10 in case of time out single shot 
 */
int   HTS221::GetHumidity(float* pfData)
{
    int16_t H_T_out, humidity_t;
    uint8_t tempReg[2] = {0,0};
    uint8_t tmp[2] = {0,0x00};
    float H_rh;
    int status; 
    status = HUM_TEMP_IO_Read(&tmp[1],  HTS221_CTRL_REG1_ADDR, 1);
#ifdef DEBUGCODE 
    if ( status ) {      printf(" error reading REG 1 \n\r");  }
    else {  printf("H success  reading REG 1 %02x \n\r", tmp[1]);  }
#endif     
    /* Output Data Rate selection */
    tmp[1] &= (HTS221_ODR_MASK);
    
    if(tmp[1] == 0x00){    
      HUM_TEMP_IO_Read(&tmp[1],  HTS221_CTRL_REG2_ADDR, 1);

      /* Serial Interface Mode selection */
      tmp[1] &= ~(HTS221_ONE_SHOT_MASK);
      tmp[1] |= HTS221_ONE_SHOT_START;
	  tmp[0] =HTS221_CTRL_REG2_ADDR;
      HUM_TEMP_IO_Write(tmp,   2);
      int nrloops=10000;  
      do{ 
        HUM_TEMP_IO_Read(&tmp[1],  HTS221_STATUS_REG_ADDR, 1);
      }while(((tmp[1] & 0x02) == 0) && nrloops--);
      if(nrloops == -1)  status|=0x10;  
	  #ifdef DEBUGCODE 
		printf("nrloopsH %d, tmp %d \n\r ", nrloops, tmp[1]);
	  #endif
    }
	
  //  if( status ==0 )
		{
        status=HUM_TEMP_IO_Read(&tempReg[0],  HTS221_HUMIDITY_OUT_L_ADDR + 0x80, 2);
#ifdef DEBUGCODE 
        if ( status ) { printf("error reading HUMIDITY_OUT  \n\r"); }
        else { printf("success  reading HUMIDITY_OUT 1 %02x %02x  \n\r",tempReg[1], tempReg[0]);  }
#endif      
    
        H_T_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
        H_rh = ((float)(H_T_out - H0_T0_out))/(H1_T0_out - H0_T0_out) * (H1_rh - H0_rh) + H0_rh;

        humidity_t = (uint16_t)(H_rh * pow((double)10,(double)HUM_DECIMAL_DIGITS));
        *pfData = ((float)humidity_t)/pow((double)10,(double)HUM_DECIMAL_DIGITS);
    
    } 
	//else {   *pfData    =-10; }
    return status ; 
}


void      HTS221::GetRawHumidity(int16_t *rawhumidity) {
    uint8_t tempReg[2] = {0,0};
    // just read the register doesn't check if the conversion is started !! 
    HUM_TEMP_IO_Read(&tempReg[0],  HTS221_HUMIDITY_OUT_L_ADDR + 0x80, 2);
    *rawhumidity= ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
}
    
    

/**
 * @brief  Read HTS221 output register, and calculate the temperature.
 * @param  pfData : Data out pointer
 * @retval  none zero in case of error 
 */
int  HTS221::GetTemperature(float* pfData) {
    int16_t T_out, temperature_t;
    uint8_t tempReg[2] = {0,0};
    uint8_t tmp[2] = {0,0x00};
    float T_degC;
    int status; 
    status=HUM_TEMP_IO_Read(&tmp[1],  HTS221_CTRL_REG1_ADDR, 1);
	if (status) return status; 
    /* Output Data Rate selection */
    tmp[1] &= (HTS221_ODR_MASK);
    
    if(tmp[1] == 0x00){
    
      status= HUM_TEMP_IO_Read(&tmp[1],  HTS221_CTRL_REG2_ADDR, 1);

      /* Serial Interface Mode selection */
      tmp[1] &= ~(HTS221_ONE_SHOT_MASK);
      tmp[1] |= HTS221_ONE_SHOT_START;
      tmp[0]=HTS221_CTRL_REG2_ADDR;
      HUM_TEMP_IO_Write(tmp,   2);
	  int nrloops=10000;
      do{
      
        HUM_TEMP_IO_Read(&tmp[1],  HTS221_STATUS_REG_ADDR, 1);
       
      }while(!(tmp[1] & 0x01) && nrloops-- );
	  if(nrloops == -1) {  status|=0x10; }	
	  #ifdef DEBUGCODE 
		printf("nrloopsT %d, tmp %d \n\r ", nrloops, tmp[1]);
	  #endif
    }

    status |= HUM_TEMP_IO_Read(&tempReg[0],  HTS221_TEMP_OUT_L_ADDR + 0x80, 2);
    T_out = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    T_degC = ((float)(T_out - T0_out))/(T1_out - T0_out) * (T1_degC - T0_degC) + T0_degC;

    temperature_t = (int16_t)(T_degC * pow((double)10,(double)TEMP_DECIMAL_DIGITS));

    *pfData = ((float)temperature_t)/pow((double)10,(double)TEMP_DECIMAL_DIGITS);
	return status; 
}


/**
 * @brief  Exit the shutdown mode for HTS221.
 * @retval None
 */
void HTS221::Power_On()
{
    uint8_t tmpReg[2];

    /* Read the register content */
    HUM_TEMP_IO_Read(&tmpReg[1],  HTS221_CTRL_REG1_ADDR, 1);

    /* Set the power down bit */
    tmpReg[1] |= HTS221_MODE_ACTIVE;
	tmpReg[0] =  HTS221_CTRL_REG1_ADDR;
    /* Write register */
    HUM_TEMP_IO_Write(tmpReg,   2);
}



/**
 * @brief  Enter the shutdown mode for HTS221.
 * @retval None
 */
void HTS221::Power_OFF()
{
    uint8_t tmpReg[2];

    /* Read the register content */
    HUM_TEMP_IO_Read(&tmpReg[1],  HTS221_CTRL_REG1_ADDR, 1);

    /* Reset the power down bit */
    tmpReg[1] &= ~(HTS221_MODE_ACTIVE);
	tmpReg[0] =  HTS221_CTRL_REG1_ADDR;
    /* Write register */
    HUM_TEMP_IO_Write(tmpReg,  2);
}


/**
 * @brief  on the heater 
 * @retval None
 */
void HTS221::Heater_On()
{
    uint8_t tmpReg[2];

    /* Read the register content */
    // HUM_TEMP_IO_Read(&tmpReg,  HTS221_CTRL_REG2_ADDR, 1); no need to read 

    /* Set the power down bit */
    // tmpReg |= HTS221_MODE_ACTIVE;
	tmpReg[0] = HTS221_CTRL_REG2_ADDR ;
    tmpReg[1] = 0x2 ; 
    /* Write register */
    HUM_TEMP_IO_Write(tmpReg,  2);
    
}

/**
 * @brief  on the heater 
 * @retval None
 */
void HTS221::Heater_Off()
{
    uint8_t tmpReg[2];

    /* Read the register content */
    // HUM_TEMP_IO_Read(&tmpReg,  HTS221_CTRL_REG2_ADDR, 1); no need to read 

    /* Set the power down bit */
    // tmpReg |= HTS221_MODE_ACTIVE;
	tmpReg[0] = HTS221_CTRL_REG2_ADDR ; 
    tmpReg[1] = 0x0 ; 
    /* Write register */
    HUM_TEMP_IO_Write(tmpReg,  2);
}


#ifdef DEBUGCODE 
DigitalOut myled(LED1);
#endif 
/*  changes 20200324    DeviceAdd is now fixed so no need to pass,  WriteAddr  is suppose to be past in pBuffer (first byte = [0])  */

int     HTS221::HUM_TEMP_IO_Write(uint8_t* pBuffer,  uint16_t NumByteToWrite){
        int status=0;
        //char wbuf[1];wbuf[0]=(char) WriteAddr;
	    status = _i2c->write(HTS221_ADDRESS , (char*) pBuffer,NumByteToWrite,false); 
/*        status |= _i2c->write( DeviceAddr, wbuf,1,true); // register write , keep writing
		
        while (NumByteToWrite--){
#ifdef DEBUGCODE 
        myled=!myled;
#endif        
            
            status |=_i2c->write( (char) * pBuffer++);
        }
        _i2c->stop();
*/
        return status; 
    }

int     HTS221::HUM_TEMP_IO_Read(uint8_t* pBuffer,  uint8_t RegisterAddr, uint16_t NumByteToRead){
        int status=0;
		char wbuf[1];wbuf[0]=(char) RegisterAddr;
       //status =_i2c->write( HTS221_ADDRESS, wbuf,1,false); // register write with stop 
       //status |= _i2c->read (HTS221_ADDRESS,(char*) pBuffer, NumByteToRead);
		status= _i2c->read_reg(HTS221_ADDRESS ,(char*) pBuffer , NumByteToRead , RegisterAddr );
        return status; 
    
    }


/* special function for debugging  */ 
#ifdef DEBUGCODE 


int HTS221::readAllReg(  ) { // uncomment only for debugging 
	uint8_t tmpReg[0x40]; // even if we don't read the first registers 
	// first register is 0x0F , to loop over the registers the MSB should be 1 but seems not to work correctly
	int status = HUM_TEMP_IO_Read( tmpReg ,   0x8F , 0x30);
	if ( status != 0 ) return status ;
	printf( "HTS221_WHO_AM_I_ADDR (0x%02X) = 0x%02X   0xBC\n\r",HTS221_WHO_AM_I_ADDR, tmpReg[HTS221_WHO_AM_I_ADDR -0XF]  );
	printf( "HTS221_RES_CONF_ADDR (0x%02X) = 0x%02X   0x1B\n\r",HTS221_RES_CONF_ADDR,tmpReg[HTS221_RES_CONF_ADDR -0XF]  );
	printf( "HTS221_INFO_L_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_INFO_L_ADDR,tmpReg[HTS221_INFO_L_ADDR -0XF]  );
	printf( "HTS221_INFO_H_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_INFO_H_ADDR,tmpReg[HTS221_INFO_H_ADDR -0XF]  );
	printf( "HTS221_CTRL_REG1_ADDR (0x%02X) = 0x%02X   0x00\n\r",HTS221_CTRL_REG1_ADDR,tmpReg[HTS221_CTRL_REG1_ADDR -0XF]  );
	printf( "HTS221_CTRL_REG2_ADDR (0x%02X) = 0x%02X   0x00\n\r",HTS221_CTRL_REG2_ADDR,tmpReg[HTS221_CTRL_REG2_ADDR -0XF]  );
	printf( "HTS221_CTRL_REG3_ADDR (0x%02X) = 0x%02X   0x00\n\r",HTS221_CTRL_REG3_ADDR,tmpReg[HTS221_CTRL_REG3_ADDR -0XF]  );
	printf( "HTS221_STATUS_REG_ADDR (0x%02X) = 0x%02X   0x00\n\r",HTS221_STATUS_REG_ADDR,tmpReg[HTS221_STATUS_REG_ADDR -0XF]  );
	printf( "HTS221_HUMIDITY_OUT_L_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_HUMIDITY_OUT_L_ADDR,tmpReg[HTS221_HUMIDITY_OUT_L_ADDR -0XF]  );
	printf( "HTS221_HUMIDITY_OUT_H_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_HUMIDITY_OUT_H_ADDR,tmpReg[HTS221_HUMIDITY_OUT_H_ADDR -0XF]  );
	printf( "HTS221_TEMP_OUT_L_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_TEMP_OUT_L_ADDR,tmpReg[HTS221_TEMP_OUT_L_ADDR -0XF]  );
	printf( "HTS221_TEMP_OUT_H_ADDR (0x%02X) = 0x%02X   0xXX\n\r",HTS221_TEMP_OUT_H_ADDR,tmpReg[HTS221_TEMP_OUT_H_ADDR -0XF]  );
	printf( "HTS221_H0_RH_X2_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H0_RH_X2_ADDR,tmpReg[HTS221_H0_RH_X2_ADDR -0XF]  );
	printf( "HTS221_H1_RH_X2_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H1_RH_X2_ADDR,tmpReg[HTS221_H1_RH_X2_ADDR -0XF]  );
	printf( "HTS221_T0_degC_X8_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T0_degC_X8_ADDR,tmpReg[HTS221_T0_degC_X8_ADDR -0XF]  );	
	printf( "HTS221_T1_degC_X8_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T1_degC_X8_ADDR,tmpReg[HTS221_T1_degC_X8_ADDR -0XF]  );	
	printf( "HTS221_T1_T0_MSB_X8_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T1_T0_MSB_X8_ADDR,tmpReg[HTS221_T1_T0_MSB_X8_ADDR -0XF]  );
	printf( "HTS221_H0_T0_OUT_L_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H0_T0_OUT_L_ADDR,tmpReg[HTS221_H0_T0_OUT_L_ADDR -0XF]  );
	printf( "HTS221_H0_T0_OUT_H_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H0_T0_OUT_H_ADDR,tmpReg[HTS221_H0_T0_OUT_H_ADDR -0XF]  );
	printf( "HTS221_H1_T0_OUT_L_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H1_T0_OUT_L_ADDR,tmpReg[HTS221_H1_T0_OUT_L_ADDR -0XF]  );
	printf( "HTS221_H1_T0_OUT_H_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_H1_T0_OUT_H_ADDR,tmpReg[HTS221_H1_T0_OUT_H_ADDR -0XF]  );
	printf( "HTS221_T0_OUT_L_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T0_OUT_L_ADDR,tmpReg[HTS221_T0_OUT_L_ADDR -0XF]  );
	printf( "HTS221_T1_OUT_H_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T1_OUT_H_ADDR,tmpReg[HTS221_T1_OUT_H_ADDR -0XF]  );
	printf( "HTS221_T1_OUT_L_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T1_OUT_L_ADDR,tmpReg[HTS221_T1_OUT_L_ADDR -0XF]  );
	printf( "HTS221_T1_OUT_H_ADDR (0x%02X) = 0x%02X   0x??\n\r",HTS221_T1_OUT_H_ADDR,tmpReg[HTS221_T1_OUT_H_ADDR -0XF]  );
	return status ; 
}	
#else	
int HTS221::readAllReg(  ) {return -1; }	 // standard implementation s
#endif
//		
    
//void      HUM_TEMP_IO_Init(void){};
//void      HUM_TEMP_IO_DeInit(void){};    
  

/**
 * @brief Set HTS221 Interrupt INT1 configuration
 * @param  HTS221_InterruptConfig_TypeDef: pointer to a HTS221_InterruptConfig_TypeDef
 *         structure that contains the configuration setting for the HTS221 Interrupt.
 * @retval None
 */
//void HTS221::INT1InterruptConfig(uint16_t Int1Config){}

/**
 * @brief  Enable INT1
 * @retval None
 */
//void HTS221::EnableIT(uint8_t IntPin){  }

/**
 * @brief  Disable  INT1
 * @retval None
 */
//void HTS221::DisableIT(uint8_t IntPin){  }

