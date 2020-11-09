/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "i2c.h"

#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"


#define I2C_SEND_TIMEOUT_RXNE_MS 5
#define I2C_SEND_TIMEOUT_TXIS_MS 5
#define I2C_SEND_TIMEOUT_SB_MS        5
#define I2C_SEND_TIMEOUT_ADDR_MS      5

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
    MX_I2C1_Init();
//main init via CubeMX or
//    LL_I2C_InitTypeDef I2C_InitStruct = {0};
//
//    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
//    /**I2C1 GPIO Configuration
//    PA9   ------> I2C1_SCL
//    PA10   ------> I2C1_SDA
//    */
//    GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    /* Peripheral clock enable */
//    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
//
//    /** I2C Initialization
//    */
//    LL_I2C_EnableAutoEndMode(I2C1);
//    LL_I2C_DisableOwnAddress2(I2C1);
//    LL_I2C_DisableGeneralCall(I2C1);
//    LL_I2C_EnableClockStretching(I2C1);
//    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
//    I2C_InitStruct.Timing = 0x2000090E;
//    I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
//    I2C_InitStruct.DigitalFilter = 0;
//    I2C_InitStruct.OwnAddress1 = 0;
//    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
//    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
//    LL_I2C_Init(I2C1, &I2C_InitStruct);
//    LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);

        LL_I2C_Enable(I2C1);
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {

  uint32_t Timeout = I2C_SEND_TIMEOUT_RXNE_MS;
  uint16_t ubReceiveIndex = 0;

  LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
  /* Loop until STOP flag is raised  */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1) && (ubReceiveIndex < count))
  {
    /* (2.1) Receive data (RXNE flag raised) **********************************/

    /* Check RXNE flag value in ISR register */
    if(LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
      /* Read character in Receive Data register.
      RXNE flag is cleared by reading data in RXDR register */
      data[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C1);

      Timeout = I2C_SEND_TIMEOUT_RXNE_MS;
    }

    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
          return true;
      }
    }
  }

  /* (3) Clear pending flags, Check Data consistency **************************/

  /* End of I2C_SlaveReceiver_MasterTransmitter_DMA Process */
  LL_I2C_ClearFlag_STOP(I2C1);
  return 0;
//    return (int8_t)HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1),
//                                          data, count, 100);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {

      /* (1) Initiate a Start condition to the Slave device ***********************/
      uint16_t  ubNbDataToTransmit = 0;
      /* Master Generate Start condition for a write request :              */
      /*    - to the Slave with a 7-Bit address                             */
      /*    - with a auto stop condition generation when transmit all bytes */
      LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

      /* (2) Loop until end of transfer received (STOP flag raised) ***************/

      uint32_t Timeout = I2C_SEND_TIMEOUT_TXIS_MS;

      /* Loop until STOP flag is raised  */
      while(!LL_I2C_IsActiveFlag_STOP(I2C1) && (ubNbDataToTransmit < count))
      {
        /* (2.1) Transmit data (TXIS flag raised) *********************************/

        /* Check TXIS flag value in ISR register */
        if(LL_I2C_IsActiveFlag_TXIS(I2C1))
        {
          /* Write data in Transmit Data register.
          TXIS flag is cleared by writing data in TXDR register */
          LL_I2C_TransmitData8(I2C1, (*data++));
          ubNbDataToTransmit++;

          Timeout = I2C_SEND_TIMEOUT_TXIS_MS;
        }

        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
          if(Timeout-- == 0)
          {
              return true;
          }
        }
      }

      /* (3) Clear pending flags, Data consistency are checking into Slave process */

      /* End of I2C_SlaveReceiver_MasterTransmitter Process */
      LL_I2C_ClearFlag_STOP(I2C1);

      return 0;
//    return (int8_t)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address << 1),
//                                           (uint8_t*)data, count, 100);
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    uint32_t msec = useconds / 1000;
    LL_mDelay(msec);
}
