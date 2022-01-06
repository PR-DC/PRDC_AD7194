/**
 * test_LCs.ino - Test AD7194
 * Author: Milos Petrasinovic <mpetrasinovic@pr-dc.com>
 * PR-DC, Republic of Serbia
 * info@pr-dc.com
 * 
 * --------------------
 * Copyright (C) 2021 PR-DC <info@pr-dc.com>
 *
 * This file is part of PRDC_AD7194.
 *
 * PRDC_AD7194 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * PRDC_AD7194 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PRDC_AD7194.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

// Library
// --------------------
// PRDC AD7194
// Author: PRDC
#include <PRDC_AD7194.h>

// Define variables
// --------------------
uint32_t t0;

// SPI communications
PRDC_AD7194 AD7194;

// setup function
// --------------------
void setup() { 
  // Communication settings
  Serial.begin(115200);
  while(!Serial.available()){};
  
  AD7194.setSPI(SPI);
  if(!AD7194.begin(PIN_SPI_SS, PIN_SPI_MISO)) {
    Serial.println(F("AD7194 initialization failed!"));
  } else {
    AD7194.printAllRegisters();
    AD7194.setClockMode(AD7194_CLK_EXT_MCLK2);
    AD7194.setRate(0x005);
    AD7194.setFilter(AD7194_MODE_SINC3);
    AD7194.enableNotchFilter(false);
    AD7194.enableChop(false);
    AD7194.rangeSetup(0, AD7194_CONF_GAIN_128); // bipolar, +-12.89 mV
    AD7194.channelSelect(AD7194_CH_0);
  }
}

// loop function
// --------------------
void loop(){
  Serial.println(micros()-t0);
  t0 = micros();
  Serial.println(AD7194.singleConversion());
  Serial.println();
}
