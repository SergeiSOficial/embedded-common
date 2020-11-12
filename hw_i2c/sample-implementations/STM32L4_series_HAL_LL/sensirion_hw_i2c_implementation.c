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

#define DMA_SEND_TIMEOUT_TC_MS 5
#define I2C_SEND_TIMEOUT_STOP_MS 5

uint32_t Timeout = 0; /* Variable used for Timeout management */
uint8_t aMasterReceiveBuffer[0xf] = {0};

__IO uint8_t ubMasterTransferComplete = 0;
__IO uint8_t ubMasterReceiveComplete = 0;

void Transfer_Complete_Callback();
void Receive_Complete_Callback();
void Transfer_Error_Callback();

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
    // MX_I2C1_Init();
    // main init via CubeMX or
    //   LL_I2C_InitTypeDef I2C_InitStruct = {0};

    //   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    //   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    //   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    //   /**I2C1 GPIO Configuration
    //   PA9   ------> I2C1_SCL
    //   PB7   ------> I2C1_SDA
    //   */
    //   GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    //   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    //   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    //   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    //   GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    //   GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    //   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //   GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    //   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    //   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    //   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    //   GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    //   GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    //   LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //   /* Peripheral clock enable */
    //   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    //   /* I2C1 DMA Init */

    //   /* I2C1_RX Init */
    //   LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_7, LL_DMA_REQUEST_3);

    //   LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7,
    //   LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    //   LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7,
    //   LL_DMA_PRIORITY_LOW);

    //   LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);

    //   LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7,
    //   LL_DMA_PERIPH_NOINCREMENT);

    //   LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7,
    //   LL_DMA_MEMORY_INCREMENT);

    //   LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);

    //   LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

    //   /* I2C1_TX Init */
    //   LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_3);

    //   LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6,
    //   LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    //   LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6,
    //   LL_DMA_PRIORITY_LOW);

    //   LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

    //   LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6,
    //   LL_DMA_PERIPH_NOINCREMENT);

    //   LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6,
    //   LL_DMA_MEMORY_INCREMENT);

    //   LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

    //   LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

    //   /** I2C Initialization
    //   */
    //   LL_I2C_EnableAutoEndMode(I2C1);
    //   LL_I2C_DisableOwnAddress2(I2C1);
    //   LL_I2C_DisableGeneralCall(I2C1);
    //   LL_I2C_EnableClockStretching(I2C1);
    //   I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    //   I2C_InitStruct.Timing = 0x2000090E;
    //   I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    //   I2C_InitStruct.DigitalFilter = 0;
    //   I2C_InitStruct.OwnAddress1 = 0;
    //   I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    //   I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    //   LL_I2C_Init(I2C1, &I2C_InitStruct);
    //   LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);

    //   /* Init with LL driver */
    //   /* DMA controller clock enable */
    //   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    //   /* DMA interrupt init */
    //   /* DMA1_Channel6_IRQn interrupt configuration */
    //   NVIC_SetPriority(DMA1_Channel6_IRQn,
    //   NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    //   NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    //   /* DMA1_Channel7_IRQn interrupt configuration */
    //   NVIC_SetPriority(DMA1_Channel7_IRQn,
    //   NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    //   NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    LL_I2C_EnableDMAReq_RX(I2C1);
    LL_I2C_EnableDMAReq_TX(I2C1);
    LL_I2C_Enable(I2C1);

    /* (2) Configure NVIC for DMA1_Channel2 and DMA1_Channel3 */

    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_6, (uint32_t)(*aMasterReceiveBuffer),
        (uint32_t)LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_TRANSMIT),
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));
    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_7,
        (uint32_t)LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_RECEIVE),
        (uint32_t) & (aMasterReceiveBuffer),
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7));

    /* (5) Enable DMA1 interrupts complete/error */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_7);
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

    address = (address << 1) & 0xff;
    ubMasterReceiveComplete = 0;
    /* (6) Configure DMA to receive data from slave
     * *****************************/
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)(data));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, count);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

    /* (7) Initiate a ReStart condition to the Slave device
     * *********************/
    /* Master Generate Start condition for a write request:
     *    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS
     *    - with a auto stop condition generation when transmit all bytes
     */
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, count,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    /* (8) Loop until end of master process completed (STOP flag raised)
     * ********/
    Timeout = I2C_SEND_TIMEOUT_STOP_MS;

    /* Loop until STOP flag is raised  */
    while ((!LL_I2C_IsActiveFlag_STOP(I2C1)) ||
           (ubMasterReceiveComplete == 0)) {
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return -1;
            }
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    return 0;
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

    address = (address << 1) & 0xff;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)(data));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, count);
    /* (2) Enable DMA transfer
     * **************************************************/
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

    /* (3) Initiate a Start condition to the Slave device
     * ***********************/

    /* Master Generate Start condition for a write request:
     *  - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS
     *  - with a auto stop condition generation when transmit all bytes
     *  - No specific answer is needed from Slave Device, configure auto-stop
     * condition
     */
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, count,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_RESTART_7BIT_WRITE);

    /* (4) Loop until end of transfer completed (DMA TC raised)
     * *****************/

    Timeout = DMA_SEND_TIMEOUT_TC_MS;

    /* Loop until DMA transfer complete event */
    while (!ubMasterTransferComplete) {
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return -1;
            }
        }
    }

    /* (5) Loop until end of master process completed (STOP flag raised)
     * ********/
    Timeout = I2C_SEND_TIMEOUT_STOP_MS;

    /* Loop until STOP flag is raised  */
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return -1;
            }
        }
    }

    /* (6) Clear pending flags, Data Command Code are checking into Slave
     * process */
    /* End of Master Process */
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Clear and Reset process variables and arrays */
    ubMasterTransferComplete = 0;

    return 0;
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

/**
 * @brief  DMA transfer complete callback
 * @note   This function is executed when the transfer complete interrupt
 *         is generated
 * @retval None
 */
void Transfer_Complete_Callback() {
    /* DMA transfer completed */
    ubMasterTransferComplete = 1;
}

/**
 * @brief  DMA receive complete callback
 * @note   This function is executed when the transfer complete interrupt
 *         is generated
 * @retval None
 */
void Receive_Complete_Callback() {
    /* DMA receive completed */
    ubMasterReceiveComplete = 1;
}

/**
 * @brief  DMA transfer error callback
 * @note   This function is executed when the transfer error interrupt
 *         is generated during DMA transfer
 * @retval None
 */
void Transfer_Error_Callback() {
}

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
    if (LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);
        Transfer_Complete_Callback();
    } else if (LL_DMA_IsActiveFlag_TE6(DMA1)) {
        LL_DMA_ClearFlag_TE6(DMA1);
        Transfer_Error_Callback();
    }
    /* USER CODE END DMA1_Channel6_IRQn 0 */

    /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

    /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel7 global interrupt.
 */
void DMA1_Channel7_IRQHandler(void) {
    /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
    if (LL_DMA_IsActiveFlag_TC7(DMA1)) {
        LL_DMA_ClearFlag_TC7(DMA1);
        Receive_Complete_Callback();
    } else if (LL_DMA_IsActiveFlag_TE7(DMA1)) {
        LL_DMA_ClearFlag_TE7(DMA1);
        Transfer_Error_Callback();
    }
    /* USER CODE END DMA1_Channel7_IRQn 0 */

    /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

    /* USER CODE END DMA1_Channel7_IRQn 1 */
}
