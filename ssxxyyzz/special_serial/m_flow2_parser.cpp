/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file m_flow2_parser.cpp
 * @author xxx.yyy
 *
 * Declarations of parser for the Lightware laser rangefinder series
 */
#include <string.h>
#include <stdlib.h>
#include "m_flow2_parser.h"


uint8_t m_flow2_parser(const uint8_t *buffer, int data_length, uint16_t *volume, uint16_t *flow, uint8_t *MFlow2_flag, int32_t MFlow2_Volume){

		//uint8_t j = 0;
		    if (data_length >= 60) {
			for(int i=0; i< data_length-1; i++){
			    if(buffer[i] == 0x3C && buffer[i+1] == 0x01 && buffer[i+2] == 0x20 ){
				(*volume) = ((uint16_t)buffer[84] << 8) | buffer[83];
			    	(*flow) = ((uint16_t)buffer[75] << 8) | buffer[74];
			    }else if(buffer[i] == 0x3C && buffer[i+1] == 0x01 && buffer[i+2] == 0x21 ){
				int32_t vol = ((uint16_t)buffer[86] << 8) | buffer[85];
				if (vol == MFlow2_Volume) {(*MFlow2_flag) = (uint8_t)2;
				} else {(*MFlow2_flag) = (uint8_t)3;}

			    }
			}

			    return buffer[84];

		    }else{
			    return -1;}

	       }
/*
uint16_t crc16_update(uint16_t crc, uint8_t data)
{
  uint16_t ret_val;
  data ^= (uint8_t) (crc) & (uint8_t) (0xFF);
  data ^= data << 4;
  ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
  ^ (uint8_t)(data >> 4)
  ^ ((uint16_t)data << 3));
  return ret_val;
}

uint16_t get_crc16z(uint8_t *p, uint16_t len)
{
  uint16_t crc16_data = 0;
  while(len--){ crc16_data = crc16_update(crc16_data,p[0]); p++;}
  return (crc16_data);
  }
*/
