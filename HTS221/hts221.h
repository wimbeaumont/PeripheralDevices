/** 
 *  hts221.h  
 *  This is a C++ class header to read the HTS221  humidity
 * ( and temperature sensor) via the I2C interface
 * substantial part of the code is copied from the AST Robotics Team 
 * see their copy right below. 
 * This program will be used to monitor the humitidy in the SOLID detector. 
 * W. Beaumont
 * (C) University Antwerpen
 * version history  
 * version 0.7  : version for debugging Linux implementation 
 * version 0.6  : added I2C status to the read / write io.  
 * version 0.5  : added heater function
 * version 0.4  : added function to read the raw humidity data and get the calibration values 
 * version 0.3  : can read the ID , read Humidity and temperature 
 *                tested with init = true in constructor
 *                 no error handling for the device in case of bussy
 *                device will keep SDA low in some cases and power cycle 
 *                seems the only option. 
 *                 no I2C error handling                            
 * version 0.1  : can read the ID 
 
 **/


/**
  ******************************************************************************
  * @file    hts221.h
  * @author  AST Robotics Team
  * @version V0.0.1
  * @date    08-April-2014
  * @brief   This file contains definitions hts221.h 
  *          firmware driver.
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
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTS221_H
#define __HTS221_H
#include "getVersion.h"

#include "stdbool.h"


#include "dev_interface_def.h"
#include "I2CInterface.h" 

#define VERSION_HTS221_HDR "0.78"


class HTS221 : public virtual getVersion {
   
    float T0_degC, T1_degC; //< Temperature in degree for calibration     
    int16_t T0_out, T1_out;//<Output temperature value for calibration
    float H0_rh, H1_rh;//< Humidity for calibration 
    int16_t H0_T0_out, H1_T0_out;//<Output Humidity value for calibration

   protected:
    /** pointer to the I2C interface driver. */
    I2CInterface* _i2c;
  uint8_t OutputDataRate;
/* HUM_TEMP sensor IO functions 
    not clear what these functions are suppose to do.  

*/
//void      HUM_TEMP_IO_Init(void);
//void      HUM_TEMP_IO_DeInit(void);

/* I2C read / wrire method  */
// returns zerro if no error 
int      HUM_TEMP_IO_Write(uint8_t* pBuffer, uint16_t NumByteToWrite);
int      HUM_TEMP_IO_Read(uint8_t* pBuffer,  uint8_t RegisterAddr, uint16_t NumByteToRead);
  
  
    
public: 

    
/**
* @constructor
* @PARM  i2cinterface   pointer to the i2c interface should not be NULL
* @PARAM init  if true the device is set to power on mode , calibration is done 
          and rate is set to 7 KHz
  @PARAM  if OneShot is true and init is true , OneShot operation is activated. 
          
*/
HTS221(I2CInterface* i2cinterface, bool init=true , bool OneShot=false);

uint8_t   ReadID(void);
void      RebootCmd(void);
void      Power_OFF(void);
void      Power_On(void);
void      Heater_On(void);
void      Heater_Off(void);
// returns non zero in case of error 
int       GetHumidity(float* pfData);
int     GetTemperature(float* pfData);

void Calibration(void);
void Init(void);
   

void getHCalValues( float & H0_rh_, float &H1_rh_, int16_t &H0_T0_out_, int16_t &H1_T0_out_){
    H0_rh_=H0_rh; H1_rh_=H1_rh;H0_T0_out_=H0_T0_out;H1_T0_out_=H1_T0_out;}

void      GetRawHumidity(int16_t * rawhumidity);

int readAllReg( void ); // only for debugging, returns -1  as default


/* Interrupt Configuration Functions 
    these are copied from the AST Robotics Team  
    not clear what these functions are suppose to do.  
    Most likley platfrom fuctions 
*/
//void      INT1InterruptConfig(uint16_t Int1Config);
//void      EnableIT(uint8_t IntPin);
//void      DisableIT(uint8_t IntPin);




 }; 
#endif /* __HTS221_H */


