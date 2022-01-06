/**
 * PRDC_AD7194.cpp - Analog Devices AD7194 ADC Library
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

#include "PRDC_AD7194.h"

// PRDC_AD7194()
// Object constructor
// --------------------
PRDC_AD7194::PRDC_AD7194() :
  _spiSettings(AD7194_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE3),
  _spi(&AD7194_DEFAULT_SPI), _CS(AD7194_DEFAULT_CS), _MISO(AD7194_DEFAULT_MISO) {

}

// setSPI() function
// Set SPI object
// --------------------
void PRDC_AD7194::setSPI(SPIClass& spi) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setSPI()"));
  #endif
  
  _spi = &spi;
}

// setSPIFrequency() function
// Set SPI frequency
// --------------------
void PRDC_AD7194::setSPIFrequency(uint32_t frequency) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setSPIFrequency()"));
  #endif
  
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE3);
}

// beginTransaction() function
// Begin SPI transaction 
// --------------------
inline void PRDC_AD7194::beginTransaction() {
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_CS, LOW);
}

// endTransaction() function
// End SPI transaction 
// --------------------
inline void PRDC_AD7194::endTransaction() {
  digitalWrite(_CS, HIGH);
  _spi->endTransaction();
}

// getRegister() function
// Reads the value of a register
// --------------------
uint32_t PRDC_AD7194::getRegister(uint8_t registerAddress, uint8_t bytesNumber) {
  #ifdef DEBUG_AD7194
    Serial.println(F("getRegister()"));
    Serial.print(F("Register "));
    Serial.println(registerAddress);
  #endif
  
  uint32_t buffer = 0x0;
  _spi->transfer(AD7194_COMM_READ | AD7194_COMM_ADDR(registerAddress));
  for(uint8_t i = 0; i < bytesNumber; i++)  {
      buffer = (buffer << 8) + _spi->transfer(0);
  }
  
  #ifdef DEBUG_AD7194
    Serial.println(buffer);
  #endif
  return buffer; 
}

// getSingleRegister() function
// Reads the value of a single register
// --------------------
uint32_t PRDC_AD7194::getSingleRegister(uint8_t registerAddress, uint8_t bytesNumber) {
  #ifdef DEBUG_AD7194
    Serial.println(F("getSingleRegister()"));
  #endif
  
  this->beginTransaction();
  uint32_t regVal = this->getRegister(registerAddress, bytesNumber);
  this->endTransaction();
  return regVal;
}

// setRegister() function
// Set the value of a register
// --------------------
void PRDC_AD7194::setRegister(uint8_t registerAddress, uint32_t value, uint8_t bytesNumber) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setRegister()"));
    Serial.print(F("Register "));
    Serial.print(registerAddress);
    Serial.print(F(" Value "));
    Serial.println(value);
  #endif
  
  uint8_t* dataPointer = (uint8_t*)&value;
  _spi->transfer(AD7194_COMM_WRITE | AD7194_COMM_ADDR(registerAddress));
  for(uint8_t i = 0; i < bytesNumber; i++)  {
    _spi->transfer(*(dataPointer + (bytesNumber-1) - i));
  }
}

// setSingleRegister() function
// Set the value of a single register
// --------------------
void PRDC_AD7194::setSingleRegister(uint8_t registerAddress, uint32_t value, uint8_t bytesNumber) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setSingleRegister()"));
  #endif
  
  this->beginTransaction();
  this->setRegister(registerAddress, value, bytesNumber);
  this->endTransaction();
}

// begin() function
// Begin communication
// --------------------
bool PRDC_AD7194::begin() {
  #ifdef DEBUG_AD7194
    Serial.println(F("begin()"));
    Serial.println(F("PRDC_AD7194 constructor instantiated successfully"));
    Serial.print(F("CS "));
    Serial.println(_CS);
    Serial.print(F("MISO "));
    Serial.println(_MISO);
  #endif
  
  _spi->begin();
  this->pinInit();
  this->reset();
  if(this->checkID()) {
    return true;
  }
  return false;
}

bool PRDC_AD7194::begin(uint8_t CS, uint8_t MISO) {
  _CS = CS;
  _MISO = MISO;
  return this->begin();
}

// end() function
// End communication
// --------------------
void PRDC_AD7194::end() {
  #ifdef DEBUG_AD7194
    Serial.println(F("end()"));
  #endif
  
  _spi->end();
}

// pinInit() function
// Pin initialization
// --------------------
void PRDC_AD7194::pinInit() {
  #ifdef DEBUG_AD7194
    Serial.println(F("pinInit()"));
  #endif

  pinMode(_CS, OUTPUT);
  digitalWrite(_CS, HIGH);
}

// reset() function
// Reset ADC
// --------------------
void PRDC_AD7194::reset() {
  #ifdef DEBUG_AD7194
    Serial.println(F("reset()"));
  #endif

  this->beginTransaction();
  _spi->transfer(0xFF);
  _spi->transfer(0xFF);
  _spi->transfer(0xFF);
  _spi->transfer(0xFF);
  _spi->transfer(0xFF);
  _spi->transfer(0xFF);
  this->endTransaction();
}

// setClockMode() function
// Set clock mode (internal or external)
// --------------------
void PRDC_AD7194::setClockMode(uint8_t clock_mode) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setClockMode()"));
  #endif
  
  _clock_mode = clock_mode;
}

// setRate() function
// Set output data rate
// --------------------
void PRDC_AD7194::setRate(uint32_t rate) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setRate()"));
  #endif
  
  _rate = rate;
}

// setFilter() function
// Set filter type (Sinc3 or Sinc4)
// --------------------
void PRDC_AD7194::setFilter(uint32_t filter) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setFilter()"));
  #endif
  
  _filter = filter;
}

// enableNotchFilter() function
// Turn on or off notch filter;
// --------------------
void PRDC_AD7194::enableNotchFilter(bool notch_state) {
  #ifdef DEBUG_AD7194
    Serial.println(F("enableNotchFilter()"));
  #endif
  
  if(notch_state) {
    _notch_filter = AD7194_MODE_REJ60;
  } else {
    _notch_filter = AD7194_MODE_NO_REJ60;
  }
}

// enableChop() function
// Turn on or off chop;
// --------------------
void PRDC_AD7194::enableChop(bool chop_state) {
  #ifdef DEBUG_AD7194
    Serial.println(F("enableChop()"));
  #endif
  
  if(chop_state) {
    _chop = AD7194_CONF_CHOP;
  } else {
    _chop = AD7194_CONF_NO_CHOP;
  }
}
// checkID() function
// Check ADC ID register
// --------------------
bool PRDC_AD7194::checkID() {
  #ifdef DEBUG_AD7194
    Serial.println(F("checkID()"));
  #endif
  
  uint8_t regVal = getSingleRegister(AD7194_REG_ID, 1);
  #ifdef DEBUG_AD7194
    Serial.print(F("ID = "));
    Serial.println(regVal & AD7194_ID_MASK);
  #endif
  return (regVal & AD7194_ID_MASK) == ID_AD7194;
}

// waitReady() function
// Waits for MISO pin to go low
// --------------------
void PRDC_AD7194::waitReady() {
  #ifdef DEBUG_AD7194
    Serial.println(F("waitReady()"));
  #endif
  while(digitalRead(_MISO)) {}
}

// setPower() function
// Set device to idle or power-down
// mode = 
//  0 - power-down
//  1 - idle
// --------------------
void PRDC_AD7194::setPower(uint8_t mode) {
  #ifdef DEBUG_AD7194
    Serial.println(F("setPower()"));
  #endif
  
   uint32_t oldPwrMode = this->getSingleRegister(AD7194_REG_MODE, 3);
   oldPwrMode &= ~(AD7194_MODE_SEL(0x7));
   uint32_t newPwrMode = oldPwrMode | 
                         AD7194_MODE_SEL((mode * (AD7194_MODE_IDLE)) |
                         (!mode * (AD7194_MODE_PWRDN)));
   this->setSingleRegister(AD7194_REG_MODE, newPwrMode, 3); 
}

// channelSelect() function
// Selects the channel to be enabled
// --------------------
void PRDC_AD7194::channelSelect(uint8_t channel) {
  #ifdef DEBUG_AD7194
    Serial.println(F("channelSelect()"));
  #endif
  uint32_t newRegValue;
  uint32_t oldRegValue = this->getSingleRegister(AD7194_REG_CONF, 3);
  oldRegValue &= ~(AD7194_CONF_CHAN(0x3FF));
  if(oldRegValue & AD7194_CONF_PSEUDO) {
    newRegValue = oldRegValue | AD7194_CONF_PD_CHAN(channel);   
  } else {
    if(channel > 15) { 
      newRegValue = oldRegValue | (1 << 16);
    } else {
      if(channel > 7) {
        channel -= 8;
      }
      newRegValue = oldRegValue | AD7194_CONF_CHAN(channel);
    }
  }
  this->setSingleRegister(AD7194_REG_CONF, newRegValue, 3);
}

// calibrate() function
// Performs the given calibration to the specified channel
// --------------------
void PRDC_AD7194::calibrate(uint8_t mode, uint8_t channel) {  
  #ifdef DEBUG_AD7194
    Serial.println(F("calibrate()"));
  #endif
  
  this->channelSelect(channel);
  uint32_t oldRegValue = this->getSingleRegister(AD7194_REG_MODE, 3);
  oldRegValue &= ~AD7194_MODE_SEL(0x7);
  uint32_t newRegValue = oldRegValue | AD7194_MODE_SEL(mode);
  this->beginTransaction();
  this->setRegister(AD7194_REG_MODE, newRegValue, 3);
  this->waitReady();
  this->endTransaction();
}

// rangeSetup() function
// Selects the polarity of the conversion and the ADC input range
// --------------------
void PRDC_AD7194::rangeSetup(uint8_t polarity, uint8_t range) {
  #ifdef DEBUG_AD7194
    Serial.println(F("rangeSetup()"));
  #endif
  
  uint32_t oldRegValue = this->getSingleRegister(AD7194_REG_CONF, 3);
  oldRegValue &= ~(AD7194_CONF_UNIPOLAR |
                   AD7194_CONF_GAIN(0x7));
  uint32_t newRegValue = oldRegValue | 
                        (polarity * AD7194_CONF_UNIPOLAR) |
                        AD7194_CONF_GAIN(range); 
  this->setSingleRegister(AD7194_REG_CONF, newRegValue, 3);

  _polarity = polarity;
  _gain = 1 << range;
}

// singleConversion() function
// Returns the result of a single conversion
// --------------------
uint32_t PRDC_AD7194::singleConversion() {
  #ifdef DEBUG_AD7194
    Serial.println(F("singleConversion()"));
  #endif
  
  uint32_t command = AD7194_MODE_SEL(AD7194_MODE_SINGLE) | 
                     AD7194_MODE_CLKSRC(_clock_mode) |
                     AD7194_MODE_RATE(_rate) |
                     _filter |
                     _notch_filter |
                     _chop;    
  this->beginTransaction();
  this->setRegister(AD7194_REG_MODE, command, 3);
  this->waitReady();
  uint32_t raw = this->getRegister(AD7194_REG_DATA, 3);
  this->endTransaction();
  
  return raw;
}

// continuousReadAverage() function
// Returns the average of several conversion results
// --------------------
uint32_t PRDC_AD7194::continuousReadAverage(uint32_t sampleNumber) {
  #ifdef DEBUG_AD7194
    Serial.println(F("continuousReadAverage()"));
  #endif
  
  uint32_t samplesAverage = 0;  
  uint32_t command = AD7194_MODE_SEL(AD7194_MODE_CONT) | 
                     AD7194_MODE_CLKSRC(_clock_mode) |
                     AD7194_MODE_RATE(_rate) | 
                     _filter |
                     _notch_filter |
                     _chop;
            
  this->beginTransaction();
  this->setRegister(AD7194_REG_MODE, command, 3);
  for(uint32_t i = 0; i < sampleNumber; i++) {
    this->waitReady();
    samplesAverage += this->getRegister(AD7194_REG_DATA, 3);
  }
  this->endTransaction();
  
  return samplesAverage/sampleNumber;
}

// temperatureRead() function
// Read data from temperature sensor and converts it to Celsius degrees
// --------------------
float PRDC_AD7194::temperatureRead() {
  #ifdef DEBUG_AD7194
    Serial.println(F("temperatureRead()"));
  #endif
  
  this->rangeSetup(0, AD7194_CONF_GAIN_1); // Bipolar operation, 0 Gain.
  this->channelSelect(AD7194_CH_TEMP);
  uint32_t dataReg = this->singleConversion();
  dataReg -= 0x800000;
  return (float)dataReg/2815.0-273; // Celsius Temperature
}

// rawToVolts() function
// Converts 24-bit raw data to volts
// --------------------
float PRDC_AD7194::rawToVolts(uint32_t raw, float vRef) {
  #ifdef DEBUG_AD7194
    Serial.println(F("rawToVolts()"));
  #endif
  
  float voltage;
  if(_polarity == 0) {
    // Bipolar mode
    voltage = (((float)raw/(1ul << 23))-1)*vRef/_gain;
  } else {
    // Unipolar mode
    voltage = ((float)raw*vRef)/(1ul << 24)/_gain;
  }
  
  return voltage;
}

// printAllRegisters() function
// Print value of each register
// --------------------
void PRDC_AD7194::printAllRegisters(void)  {
  Serial.println(F("Read all registers"));
  this->beginTransaction();
  Serial.println(F("Register 0"));
  Serial.println(this->getRegister(0, 1));
  Serial.println(F("Register 1"));
  Serial.println(this->getRegister(1, 3));
  Serial.println(F("Register 2"));
  Serial.println(this->getRegister(2, 3));
  Serial.println(F("Register 3"));
  Serial.println(this->getRegister(3, 3));
  Serial.println(F("Register 4"));
  Serial.println(this->getRegister(4, 1));
  Serial.println(F("Register 5"));
  Serial.println(this->getRegister(5, 1));
  Serial.println(F("Register 6"));
  Serial.println(this->getRegister(6, 3));
  Serial.println(F("Register 7"));
  Serial.println(this->getRegister(7, 3));
  this->endTransaction();
}