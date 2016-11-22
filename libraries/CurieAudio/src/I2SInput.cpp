//***************************************************************
//
// Copyright (c) 2016 Intel Corporation.  All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
//
//***************************************************************

#include "I2SInput.h"
#include <interrupt.h>
#include "soc_dma.h"
#include "soc_i2s.h"
#include "variant.h"

const int numPpBuffer = 2;
const int NUMBER_OF_SAMPLES = 256;

int fullBufferIndex;
int fillBufferIndex;
    
static i2sErrorCode kickOff(void);

static struct pingpongstruct
{
    int32_t buf[NUMBER_OF_SAMPLES];  // Left and right samples
    int32_t *bufferReadPoint;
    int32_t *bufferEndPoint;
    uint16_t full;
} rxPpBuffer[numPpBuffer];

I2SInputClass I2SInput;

void rxi2s_done(void *x)
{
    rxPpBuffer[fillBufferIndex].full = 1;
    fillBufferIndex = (fillBufferIndex + 1) & 0x01;

    if (rxPpBuffer[fillBufferIndex].full)
    {
        I2SInput.rx_buffer_over_written++;
        //Serial.println("**rx_buffer_over_written*");
        //Serial.println(I2SInput.rx_buffer_over_written);
    }
    rxPpBuffer[fillBufferIndex].full = 0;

    if (I2SInput.userCB) I2SInput.userCB();
}

void rxi2s_err(void *x)
{
    I2SInput.rxerror_flag = 1;
    soc_i2s_stop_receive();
    kickOff();
}

static i2sErrorCode kickOff(void)
{
    void *bufPtrArray[numPpBuffer];
    int i;

    for (i = 0; i < numPpBuffer; i++)
    {
        bufPtrArray[i] = &rxPpBuffer[i].buf[0];
        rxPpBuffer[i].full = 0;
        rxPpBuffer[i].bufferReadPoint = &rxPpBuffer[i].buf[0];
        rxPpBuffer[i].bufferEndPoint = &rxPpBuffer[i].buf[NUMBER_OF_SAMPLES];
    }

    fullBufferIndex = fillBufferIndex = 0;

    if (soc_i2s_receive_loop(bufPtrArray, numPpBuffer,
                             sizeof(rxPpBuffer[0].buf),
                             sizeof(int32_t)) != DRV_RC_OK)
    {
        return I2S_READ_DRIVER_FAIL;
    }

    return SUCCESS;
}

I2SInputClass::I2SInputClass()
{
    rx_buffer_over_written = 0;
    rxerror_flag = 0;
    userCB = NULL;

    for (int i = 0; i < numPpBuffer; i++)
    {
        rxPpBuffer[i].full = 0;
        rxPpBuffer[i].bufferReadPoint = &rxPpBuffer[i].buf[0];
        rxPpBuffer[i].bufferEndPoint = &rxPpBuffer[i].buf[NUMBER_OF_SAMPLES];
    }
    fullBufferIndex = fillBufferIndex = 0;
}

i2sErrorCode I2SInputClass::begin(curieI2sSampleRate sampleRate,
                                  curieI2sSampleSize resolution,
                                  curieI2sMode mode, uint8_t master)
{
    struct soc_i2s_cfg rxcfg;

    sampleSize = resolution;
    muxRX(1);
    soc_i2s_init(I2S_CHANNEL_RX);
    soc_dma_init();

    rxcfg.clk_divider = sampleRateMap[(resolution * MAX_I2S_RATE) + sampleRate];
    rxcfg.resolution = sampleSizeMap[resolution];
    rxcfg.mode = (uint8_t)mode;
    rxcfg.master = master;

    rxcfg.cb_done = rxi2s_done;
    rxcfg.cb_err = rxi2s_err;

    if (soc_i2s_config(I2S_CHANNEL_RX, &rxcfg) != DRV_RC_OK)
        return I2S_INIT_FAIL;

    kickOff();
    delay(1000);

    return SUCCESS;
}

void I2SInputClass::end()
{
    soc_i2s_stop_receive();
    muxRX(0);
}

i2sErrorCode I2SInputClass::read(int32_t buffer[], uint32_t length,
                                 uint32_t blocking)
{
    int bytesLeft = length;
    int buffRead = 0;
       
    if(fullBufferIndex == fillBufferIndex)
        fullBufferIndex = (fillBufferIndex + 1) & 0x01;
           
    do
    {
        if (rxPpBuffer[fullBufferIndex].full)
        {
            int bytesAvailable = rxPpBuffer[fullBufferIndex].bufferEndPoint -
                                 rxPpBuffer[fullBufferIndex].bufferReadPoint;
            int bytesToRead =
                bytesLeft > bytesAvailable ? bytesAvailable : bytesLeft;                
            
            memcpy(&buffer[buffRead], rxPpBuffer[fullBufferIndex].bufferReadPoint,
                   bytesToRead*4); 
            buffRead += bytesToRead;      
            bytesLeft -= bytesToRead;
            
            
            rxPpBuffer[fullBufferIndex].bufferReadPoint += bytesToRead;
            if (rxPpBuffer[fullBufferIndex].bufferReadPoint ==
                rxPpBuffer[fullBufferIndex].bufferEndPoint)
            {
                rxPpBuffer[fullBufferIndex].bufferReadPoint =
                    &rxPpBuffer[fullBufferIndex].buf[0];
                rxPpBuffer[fullBufferIndex].full = 0;
                fullBufferIndex = (fullBufferIndex + 1) & 0x1;
            }
        }

        if (rxerror_flag == 1)
        {
            return I2S_READ_FAIL;
        }
    } while (blocking && (bytesLeft > 0));

    return SUCCESS;
}

i2sErrorCode I2SInputClass::read(int32_t *leftSample, int32_t *rightSample,
                                 uint32_t blocking)
{
    do
    {
        if (rxPpBuffer[fullBufferIndex].full)
        {
            *leftSample = *rxPpBuffer[fullBufferIndex].bufferReadPoint++;
            *rightSample = *rxPpBuffer[fullBufferIndex].bufferReadPoint++;
            if (rxPpBuffer[fullBufferIndex].bufferReadPoint ==
                rxPpBuffer[fullBufferIndex].bufferEndPoint)
            {
                rxPpBuffer[fullBufferIndex].bufferReadPoint =
                    &rxPpBuffer[fullBufferIndex].buf[0];
                rxPpBuffer[fullBufferIndex].full = 0;
                fullBufferIndex = (fullBufferIndex + 1) & 0x1;
            }
            break;
        }

        if (rxerror_flag == 1)
        {
            return I2S_READ_FAIL;
        }
    } while (blocking);

    return SUCCESS;
}
