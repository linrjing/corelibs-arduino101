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

//CurieI2SDMA.h

#ifndef _CURIEI2SDMA_H_
#define _CURIEI2SDMA_H_

#include <Arduino.h>

//I2S Arduino Pins
#define I2S_TXD     7
#define I2S_TWS     4
#define I2S_TSCK    2
#define I2S_RXD     5
#define I2S_RWS     3
#define I2S_RSCK    8

#define I2S_DMA_OK 0
#define I2S_DMA_FAIL 1

#define BUFF_SIZE_TEMP	256

typedef enum
{
  PHILIPS_MODE,
  RIGHT_JST_MODE,
  LEFT_JST_MODE,
  DSP_MODE
} i2s_mode_t;

class Curie_I2SDMA
{
	private:
 
		// initializes i2s and DMA interface
		int ini();
   
		// mux/demux the i2s rx pins into i2s mode
		void muxRX(bool enable);
        
		// mux/demux the i2s tx pins into i2s mode
		void muxTX(bool enable);   
	
	public:
		Curie_I2SDMA();
   
    		//setup the I2S hardware in the specified mode, sample rate, bits per sample and master/slave mode.
    		int begin(int mode = PHILIPS_MODE, int sample_rate = 44100, long resolution = 16, uint8_t master = 2);
    
    		// alternative to above
    		int beginPhilips(long sampleRate, int bitsPerSample); 
    		int beginRightJustified(long sampleRate, int bitsPerSample); 
    		int beginLeftJustified(long sampleRate, int bitsPerSample);
    		int beginDsp(long sampleRate, int bitsPerSample);
    
    		// write count samples, returns the number of samples written
    		int writeSamples(const uint8_t data[], int count); // 8-bit
    		int writeSamples(const uint16_t data[], int count); // 16-bit
    		int writeSamples(const uint32_t data[], int count); // 32-bit
    		size_t writeSamples(const uint8_t *buffer, size_t size); // 12/16/24/32-bit
    		size_t writeSamples(uint8_t);

    		// read count samples, returns the number of samples read
    		int readSamples(uint8_t data[], int count); // 8-bit
    		int readSamples(uint16_t data[], int count); // 16-bit
    		int readSamples(uint32_t data[], int count); // 32-bit
    
   		 // stop the I2S and DMA hardware         	
    		void end(); 
    
    		// check whether the TX/RX are allowed to write/read data
    		size_t availableForWrite();
    		size_t availableForRead();      
       
    		// call backs
    		void onReceive(void(*)(int)); // add an event handler for when data is received
    		void onTransmit(void(*)(int)); // add an event handler for when data is transmitted

    		// from Stream (these will start RX mode, if not enabled)
    		virtual int available();
    		virtual int read();
    		virtual int peek();
    		virtual void flush(); 
    
    		// override default pins, must be called before begin(...) (optional)
    		void setTxPins(int clock, int wordSelect, int data);
    		void setRxPins(int clock, int wordSelect, int data);
};

extern Curie_I2SDMA CurieI2SDMA;

#endif
