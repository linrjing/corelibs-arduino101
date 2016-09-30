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
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//***************************************************************

//CurieI2SDMA.cpp

#include "CurieI2SDMA.h"
#include "soc_i2s.h"
#include "soc_dma.h"
#include "variant.h"
#include <interrupt.h>

static void txi2s_done(void* x);
static void rxi2s_done(void* x);
static void txi2s_err(void* x);
static void rxi2s_err(void* x);

volatile uint8_t txdone_flag = 0; // record whether the tx is finished successfully
volatile uint8_t txerror_flag = 0; // record whether the tx is something wrong

volatile uint8_t rxdone_flag = 0; // record whether the rx is finished successfully
volatile uint8_t rxerror_flag = 0; // record whether the rx is something wrong

volatile uint8_t txfirst_flag = 1; // once the tx side been called and worked OK, txfirst_flag will be set 0.
volatile uint8_t rxfirst_flag = 1; // once the rx side been called and worked OK, txfirst_flag will be set 0.

uint8_t frameDelay = 0;

uint16_t write_buff_16[BUFF_SIZE_TEMP];
uint32_t write_buff_32[BUFF_SIZE_TEMP];
uint16_t read_buff_16[BUFF_SIZE_TEMP+2]; // there are two extra zero need to be received.
uint32_t read_buff_32[BUFF_SIZE_TEMP+2];

uint8_t *record_buff_adress_8=NULL;
uint16_t *record_buff_adress_16=NULL;
uint32_t *record_buff_adress_32=NULL;

volatile uint32_t tx_buf_size = 0; // recode the number of data on tx side.
volatile uint32_t rx_buf_size = 0; // recode the number of data on tx side.

struct soc_i2s_cfg cfg[2];

void (*i2s_txDoneCB)(int);
void (*i2s_rxDoneCB)(int);

int recode_data()
{
	int count = rx_buf_size;
  
	// if the type of data is uint8_t
	if(record_buff_adress_8 != NULL && record_buff_adress_16 == NULL && record_buff_adress_32 == NULL)
	{
		int count_data = count*8/cfg[I2S_CHANNEL_RX].resolution+2;
		int non_zero = 0;
		while(0 == read_buff_32[non_zero] && non_zero < count_data)
		non_zero++;
		if(non_zero == count_data)
			return I2S_DMA_FAIL;  
       
		if(12 == cfg[I2S_CHANNEL_RX].resolution )
		{
			for(int i = 0; i < count_data-non_zero && (3*i/2) < count; ++i)
			{
				if(i%2 == 0)
					record_buff_adress_8[3*i/2] = (uint8_t)(read_buff_32[i+non_zero] & 0xFF);
        			else
        			{
					record_buff_adress_8[3*(i-1)/2+1] 
						= (uint8_t)(((read_buff_32[i+non_zero] & 0xFF)<<4)+((read_buff_32[i+non_zero-1] & 0xF00)>>8));
					record_buff_adress_8[3*(i-1)/2+2] 
						= (uint8_t)((read_buff_32[i+non_zero] & 0xFF0)>>4);
				}      
			}
		}
		else if(24 == cfg[I2S_CHANNEL_RX].resolution )
		{
			for(int i = 0; i < count_data-non_zero && (3*i) < count; ++i)
			{
				record_buff_adress_8[3*i] = (uint8_t)(read_buff_32[i+non_zero] & 0xFF);
				record_buff_adress_8[3*i+1] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF00)>>8);
				record_buff_adress_8[3*i+2] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF0000)>>16);       
			}
		}
		else if(16 == cfg[I2S_CHANNEL_TX].resolution )
		{
			for(int i = 0; i < count_data-non_zero && (2*i) < count; ++i)
			{
				record_buff_adress_8[2*i] = (uint8_t)(read_buff_32[i+non_zero] & 0xFF);
				record_buff_adress_8[2*i+1] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF00)>>8);     
			}
		}
		else if(32 == cfg[I2S_CHANNEL_TX].resolution )
		{
			for(int i = 0; i < count_data-non_zero && (4*i) < count; ++i)
			{
				record_buff_adress_8[4*i] = (uint8_t)(read_buff_32[i+non_zero] & 0xFF);
				record_buff_adress_8[4*i+1] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF00)>>8);
				record_buff_adress_8[4*i+2] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF0000)>>16);
				record_buff_adress_8[4*i+3] = (uint8_t)((read_buff_32[i+non_zero] & 0xFF000000)>>24);        
			}
		}
		else
		{
			return I2S_DMA_FAIL;
		}   
	}
	// if the type of data is uint16_t
	else if(record_buff_adress_8 == NULL && record_buff_adress_16 != NULL && record_buff_adress_32 == NULL)
	{
		int count_data = count + 2;
		int non_zero = 0;
		while(0 == read_buff_16[non_zero] && non_zero < count_data)
			non_zero++;
		if(non_zero == count_data)
			return I2S_DMA_FAIL;
      
		for(int i = 0;i <count_data - non_zero && i < count;++i)
			record_buff_adress_16[i] = read_buff_16[i+non_zero];         
	}
	// if the type of data is uint32_t
	else if(record_buff_adress_8 == NULL && record_buff_adress_16 == NULL && record_buff_adress_32 != NULL)
	{
		int count_data = count + 2;
		int non_zero = 0;
		while(0 == read_buff_32[non_zero] && non_zero < count_data)
		non_zero++;
		if(non_zero == count_data)
			return I2S_DMA_FAIL;

		for(int i = 0;i <count_data - non_zero && i < count;++i)
			record_buff_adress_32[i] = read_buff_32[i+non_zero];         
	}
	else
		return I2S_DMA_FAIL;
    
	return I2S_DMA_OK;     
} 
      
static void txi2s_done(void* x)
{
	delay(frameDelay);
	if(i2s_txDoneCB != NULL)
	{
		i2s_txDoneCB(tx_buf_size);		
	} 
	txdone_flag = 1;

	return;
}

static void rxi2s_done(void* x)
{
	recode_data();
	if(i2s_rxDoneCB != NULL)
	{
		i2s_rxDoneCB(rx_buf_size);		
	} 
	rxdone_flag = 1;

	return;
}

static void txi2s_err(void* x)
{
	txerror_flag = 1;

	return;
}

static void rxi2s_err(void* x)
{
	rxerror_flag = 1;
    
	return;
}

Curie_I2SDMA CurieI2SDMA;

Curie_I2SDMA::Curie_I2SDMA()
{
	i2s_txDoneCB = NULL;
	i2s_rxDoneCB = NULL;
}

int Curie_I2SDMA::ini()
{
	muxTX(1);
	muxRX(1);
	soc_i2s_init();
	soc_dma_init();
	return I2S_DMA_OK; 
}

void Curie_I2SDMA::muxTX(bool enable)
{
	int mux_mode = GPIO_MUX_MODE;
	if(enable)
	{
		mux_mode = I2S_MUX_MODE;
	}
    
	/* Set SoC pin mux configuration */
	SET_PIN_MODE(g_APinDescription[I2S_TXD].ulSocPin, mux_mode);
	SET_PIN_MODE(g_APinDescription[I2S_TWS].ulSocPin, mux_mode);
	SET_PIN_MODE(g_APinDescription[I2S_TSCK].ulSocPin, mux_mode);
	g_APinDescription[I2S_TXD].ulPinMode = mux_mode;
	g_APinDescription[I2S_TWS].ulPinMode = mux_mode;
	g_APinDescription[I2S_TSCK].ulPinMode  = mux_mode;
}

void Curie_I2SDMA::muxRX(bool enable)
{
	int mux_mode = GPIO_MUX_MODE;
	if(enable)
	{
		mux_mode = I2S_MUX_MODE;
	}
    
	/* Set SoC pin mux configuration */
	SET_PIN_MODE(49, mux_mode); //I2S_RXD
	SET_PIN_MODE(51, mux_mode); //I2S_RWS
	SET_PIN_MODE(50,  mux_mode); //I2S_RSCK
	g_APinDescription[I2S_RXD].ulPinMode = mux_mode;
	g_APinDescription[I2S_RWS].ulPinMode = mux_mode;
	g_APinDescription[I2S_RSCK].ulPinMode  = mux_mode;
}

int Curie_I2SDMA::begin(int mode,int sample_rate,long resolution,uint8_t master)
{
  if(I2S_DMA_OK != ini())
    return I2S_DMA_FAIL;

	switch(mode)
	{
		case PHILIPS_MODE:
			mode = I2S_MODE_PHILLIPS ;
			break;
		case RIGHT_JST_MODE:
			mode = I2S_MODE_RJ;
			break;
		case LEFT_JST_MODE:
			mode = I2S_MODE_LJ;
			break;
		case DSP_MODE:
			mode = I2S_MODE_DSP;
			break;
		default:
			break;
	}
    
	cfg[I2S_CHANNEL_TX].sample_rate = sample_rate;
	cfg[I2S_CHANNEL_TX].resolution = resolution;
	cfg[I2S_CHANNEL_TX].mode = mode;
	cfg[I2S_CHANNEL_TX].master = master;
 
 	cfg[I2S_CHANNEL_TX].cb_done = txi2s_done;
	cfg[I2S_CHANNEL_TX].cb_err = txi2s_err; 	
	txdone_flag = 0;
	txerror_flag = 0;
 
	cfg[I2S_CHANNEL_RX].sample_rate = sample_rate;
	cfg[I2S_CHANNEL_RX].resolution = resolution;
	cfg[I2S_CHANNEL_RX].mode = mode;
	cfg[I2S_CHANNEL_RX].master = master;

 	cfg[I2S_CHANNEL_RX].cb_done = rxi2s_done;
	cfg[I2S_CHANNEL_RX].cb_err = rxi2s_err; 	
	rxdone_flag = 0;
	rxerror_flag = 0;
    
	if(0 == master || 1 == master)
	{
		soc_i2s_config(I2S_CHANNEL_TX, &cfg[I2S_CHANNEL_TX]);
		soc_i2s_config(I2S_CHANNEL_RX, &cfg[I2S_CHANNEL_RX]);
	}
	else
	{
		cfg[I2S_CHANNEL_TX].master = 1;
		soc_i2s_config(I2S_CHANNEL_TX, &cfg[I2S_CHANNEL_TX]);
		cfg[I2S_CHANNEL_RX].master = 0;
		soc_i2s_config(I2S_CHANNEL_RX, &cfg[I2S_CHANNEL_RX]);
	}  
	frameDelay = cfg[I2S_CHANNEL_TX].sample_rate*32*4/1000;
   
	return I2S_DMA_OK;
}

int Curie_I2SDMA::beginPhilips(long sampleRate, int bitsPerSample)
{
 	return begin(PHILIPS_MODE,sampleRate,bitsPerSample);
}

int Curie_I2SDMA::beginRightJustified(long sampleRate, int bitsPerSample)
{
	return begin(RIGHT_JST_MODE,sampleRate,bitsPerSample);
}

int Curie_I2SDMA::beginLeftJustified(long sampleRate, int bitsPerSample)
{
	return begin(LEFT_JST_MODE,sampleRate,bitsPerSample);
}

int Curie_I2SDMA::beginDsp(long sampleRate, int bitsPerSample)
{
	return begin(DSP_MODE,sampleRate,bitsPerSample);
}

int Curie_I2SDMA::writeSamples(const uint8_t data[], int count)
{
	return writeSamples(data, (size_t)count);
}

int Curie_I2SDMA::writeSamples(const uint16_t data[], int count)
{
	txdone_flag = 0;
	txerror_flag = 0;
	tx_buf_size = count;
	int status = soc_i2s_stream(data, count*sizeof(uint16_t),sizeof(uint16_t),0); 
	if(status)
		return 0;
	else
 		return count;
}

int Curie_I2SDMA::writeSamples(const uint32_t data[], int count)
{
	txdone_flag = 0;
	txerror_flag = 0;
	tx_buf_size = count;
	int status = soc_i2s_stream(data, count*sizeof(uint32_t),sizeof(uint32_t),0); 
 	if(status)
		return 0;
	else
		return count;

}

size_t Curie_I2SDMA::writeSamples(uint8_t data)
{
	txdone_flag = 0;	
	txerror_flag = 0;
	uint32_t temp[1] = {data};
	tx_buf_size = 1;
	size_t status = writeSamples(temp, 1);
 	if(status)
		return 0;
	else
		return 1;
}

size_t Curie_I2SDMA::writeSamples(const uint8_t *samples, size_t size)
{
	txdone_flag = 0;
	txerror_flag = 0;
	int count_data = size*8/cfg[I2S_CHANNEL_TX].resolution;
	size_t status = 0;
	if(12 == cfg[I2S_CHANNEL_TX].resolution )
	{
		for(int i = 0; i < count_data; ++i)
		{
			if(i%2 == 0)
				write_buff_16[i] = (*(samples+3*i/2)<<0) + ((*(samples+3*i/2+1) & 0xF)<<8);
			else
				write_buff_16[i] = ((*(samples+3*(i-1)/2+1) &0xF0)>>4) + (*(samples+3*(i-1)/2+2) << 4);
		}
		status = writeSamples(write_buff_16, count_data);
	}
	else if(24 == cfg[I2S_CHANNEL_TX].resolution )
	{
		for(int i = 0; i < count_data; ++i)
		{
			write_buff_32[i] = (*(samples+3*i)<<0)+(*(samples+3*i+1)<<8)+(*(samples+3*i+2)<<16);
		}
		status = writeSamples(write_buff_32, count_data);
	}
	else if(16 == cfg[I2S_CHANNEL_TX].resolution )
	{
		for(int i = 0; i < count_data; ++i)
		{
			write_buff_16[i] = (*(samples+2*i)<<0)+(*(samples+2*i+1)<<8);
		}
		status = writeSamples(write_buff_16, count_data);
	}
	else if(32 == cfg[I2S_CHANNEL_TX].resolution )
	{
		for(int i = 0; i < count_data; ++i)
		{
			write_buff_32[i] = (*(samples+4*i)<<0)+(*(samples+4*i+1)<<8)+(*(samples+4*i+2)<<16)+(*(samples+4*i+3)<<24);
		}
		status = writeSamples(write_buff_32, count_data);
	}
	else
		return 0;
	tx_buf_size = size;
      
	if(status)
		return 0;
	else
		return size;      
}

int Curie_I2SDMA::readSamples(uint8_t data[], int count)
{
	rxdone_flag = 0;
	rxerror_flag = 0;
	record_buff_adress_8 = data;

	int count_data = count*8/cfg[I2S_CHANNEL_RX].resolution+2;
  
	int status = soc_i2s_listen(read_buff_32, (count_data)*sizeof(uint32_t),sizeof(uint32_t),0); 
	if(status)
		return 0;

	rx_buf_size = count; 
    
 	if(status)
		return 0;
	else
  		return count; 
}

int Curie_I2SDMA::readSamples(uint16_t data[], int count)
{
	rxdone_flag = 0;
	rxerror_flag = 0;
	if(record_buff_adress_8 == NULL)
		record_buff_adress_16 = data;
	rx_buf_size = count;  
	int count_data = count + 2;
	int status = soc_i2s_listen(read_buff_16, (count_data)*sizeof(uint16_t),sizeof(uint16_t),0); 
   
 	if(status)
		return 0;
	else
		return count; 
}

int Curie_I2SDMA::readSamples(uint32_t data[], int count)
{
	rxdone_flag = 0;
	rxerror_flag = 0;
	if(record_buff_adress_8 == NULL)
		record_buff_adress_32 = data;
	rx_buf_size = count;  
	int count_data = count + 2;
	int status = soc_i2s_listen(read_buff_32, (count_data)*sizeof(uint32_t),sizeof(uint32_t),0);
   
 	if(status)
		return 0;
	else
  		return count; 

}

void Curie_I2SDMA::end()
{
	soc_i2s_stop_stream();
	soc_i2s_stop_listen();
	muxTX(0);
	muxRX(0); 
}
size_t Curie_I2SDMA::availableForWrite()
{
	if(txfirst_flag)
	{
		txfirst_flag = 0;
		return BUFF_SIZE_TEMP;
	}
	uint32_t reg = soc_i2s_read_fifo(I2S_CHANNEL_TX);
	if(txdone_flag == 1 &&  reg == 0)
		return BUFF_SIZE_TEMP;
	else
		return 0;    
}

size_t Curie_I2SDMA::availableForRead()
{
	if(rxfirst_flag)
	{
		rxfirst_flag = 0;
		return BUFF_SIZE_TEMP;
	}
	uint32_t reg = soc_i2s_read_fifo(I2S_CHANNEL_RX);

	if(rxdone_flag == 1 &&  reg == 0)
		return BUFF_SIZE_TEMP;
	else
		return 0;    
}

int Curie_I2SDMA::available()
{
	return 0;
}

int Curie_I2SDMA::read()
{
	return 0;
}

int Curie_I2SDMA::peek()
{
	return 0;
}

void Curie_I2SDMA::flush()
{
}

void Curie_I2SDMA::onTransmit(void(* userCallBack)(int))
{
	i2s_txDoneCB = userCallBack;
}

void Curie_I2SDMA::onReceive(void(* userCallBack)(int))
{
	i2s_rxDoneCB = userCallBack;
}



void Curie_I2SDMA::setTxPins(int clock, int wordSelect, int data)
{
}
void Curie_I2SDMA::setRxPins(int clock, int wordSelect, int data)
{
}
