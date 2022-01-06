/**
 * test_AD7193.ino - Test AD7194
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

// SPI communications
PRDC_AD7194 AD7194;

// setup function
// --------------------
void setup() { 
  // Communication settings
  Serial.begin(1000000);
  AD7194.setSPI(SPI);

  if(!AD7194.begin(PIN_SPI_SS, PIN_SPI_MISO)) {
    Serial.println(F("AD7194 initialization failed!"));
  }
}

// loop function
// --------------------
void loop(){
  Serial.println(AD7194.checkID());
  delay(500);
}