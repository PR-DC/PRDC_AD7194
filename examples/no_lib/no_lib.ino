/**
 * no_lib.ino - Test AD7194 without lib
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
#include "SPI.h"

// Define variables
// --------------------
#define SPI_CS PA4
#define AD7194_REG_ID 4
#define AD7194_COMM_READ (1 << 6)
#define AD7194_COMM_ADDR(x) (((x) & 0x7) << 3)

// setup function
// --------------------
void setup() {
  // Define pin modes
  pinMode(SPI_CS, OUTPUT);
  
  // Communication settings
  Serial.begin(115200);
  SPI.begin();
  
  // Reset ADC
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();
}

// loop function
// --------------------
void loop() {
  // Check ADC ID
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS, LOW);
  unsigned char outByte = AD7194_COMM_READ | AD7194_COMM_ADDR(AD7194_REG_ID);
  SPI.transfer(outByte);
  unsigned char inByte = SPI.transfer(0);
  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();
  Serial.println(inByte & 0xF);
  delay(500);
}
