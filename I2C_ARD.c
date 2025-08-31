#include <Wire.h>
#include <math.h>
#include "sunsensorestimation1.h"


// ---- Device ----
const uint8_t ADR  = 0x13;   // 7-bit I2C address
const float   VREF = 3.300;  // set to your board's reference/AVDD

// ---- Register addresses (ADS7138) ----
const uint8_t REG_SYSTEM_STATUS   = 0x00;  
const uint8_t REG_GENERAL_CFG     = 0x01;  
const uint8_t REG_DATA_CFG        = 0x02;  
const uint8_t REG_OSR_CFG         = 0x03;  
const uint8_t REG_OPMODE_CFG      = 0x04;  
const uint8_t REG_PIN_CFG         = 0x05;  
const uint8_t REG_SEQUENCE_CFG    = 0x10;  
const uint8_t REG_MANUAL_CH_SEL   = 0x11;  

// RECENT_x LSB starts at 0xA0 and goes LSB,MSB pairs
const uint8_t RECENT_BASE_LSB = 0xA0;

// ---- Command opcodes ----
const uint8_t OPC_READ_REG_1   = 0x10;
const uint8_t OPC_WRITE_REG_1  = 0x08;
const uint8_t OPC_READ_BLOCK   = 0x30;
const uint8_t OPC_SET_BITS     = 0x18;
const uint8_t OPC_CLEAR_BITS   = 0x20;

// ---- Low-level helpers ----
static inline bool regWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADR);
  Wire.write(OPC_WRITE_REG_1);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static inline bool regRead1(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(ADR);
  Wire.write(OPC_READ_REG_1);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; 
  if (Wire.requestFrom((int)ADR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

static inline bool regReadN(uint8_t startReg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(ADR);
  Wire.write(OPC_READ_BLOCK);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static inline bool regSetBits(uint8_t reg, uint8_t mask) {
  Wire.beginTransmission(ADR);
  Wire.write(OPC_SET_BITS);
  Wire.write(reg);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

static inline bool regClearBits(uint8_t reg, uint8_t mask) {
  Wire.beginTransmission(ADR);
  Wire.write(OPC_CLEAR_BITS);
  Wire.write(reg);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

// ---- Convert helpers ----
static inline bool readRecent12(uint8_t ch, uint16_t &code12) {
  uint8_t buf[2];
  uint8_t lsbAddr = RECENT_BASE_LSB + (ch * 2);
  if (!regReadN(lsbAddr, buf, 2)) return false;  
  uint16_t raw16 = ((uint16_t)buf[1] << 8) | buf[0];
  code12 = (raw16 >> 4) & 0x0FFF;
  return true;
}

// ---- CH0 sıcaklık hesaplama fonksiyonu ----
float voltageToTempC(float V_meas) {
  // V_meas mV cinsinden olmalı
  float a = -0.00433;
  float b = -13.582;
  float c = 2230.8 - V_meas;

  float D = b*b - 4*a*c;
  if (D < 0) return -999.0; // geçersiz

  float x1 = (-b + sqrt(D)) / (2*a);
  float x2 = (-b - sqrt(D)) / (2*a);

  float T1 = 30 + x1;
  float T2 = 30 + x2;

  if (T1 > -40 && T1 < 125) return T1;
  else return T2;
}

void configureManual() {
  regSetBits(REG_GENERAL_CFG, 0x01);  
  delay(2);

  regWrite(REG_PIN_CFG, 0x00);        
  regWrite(REG_DATA_CFG, 0x00);       
  regWrite(REG_OSR_CFG,  0x00);       
  regWrite(REG_OPMODE_CFG, 0x00);     
  regWrite(REG_SEQUENCE_CFG, 0x00);   
  regSetBits(REG_GENERAL_CFG, 0x20);  
}

void setup() {
  Serial.begin(115200);
  Wire.begin();              
  Wire.setClock(100000);     

  configureManual();

  uint8_t sys=0;
  if (regRead1(REG_SYSTEM_STATUS, sys)) {
    Serial.print(F("SYSTEM_STATUS=0x")); Serial.println(sys, HEX);
  } else {
    Serial.println(F("SYSTEM_STATUS read error"));
  }

  Serial.println(F("Manual per-channel conversions (CH0..CH5)"));
}

void loop() {
  double y[5];   // CH1–CH5 değerleri
  double result[3];

  for (uint8_t ch = 0; ch <= 5; ch++) {
    uint32_t sum = 0;
    uint16_t code;

    for (uint8_t i = 0; i < 10; i++) {
      regWrite(REG_MANUAL_CH_SEL, (uint8_t)(ch));
      regSetBits(REG_GENERAL_CFG, 0x08);
      delayMicroseconds(2000);

      if (readRecent12(ch, code)) {
        sum += code;
      }
    }

    uint16_t avgCode = sum / 10;
    float volts = (avgCode / 4095.0f) * VREF;

    if (ch >= 1 && ch <= 5) {
      y[ch - 1] = volts;  
    }
  }

  // 1x3 sonuç matrisi
  sunsensorestimation1(y, result);

  // Seri porta yazdır
  Serial.print("sunsensorestimation1(y) = [");
  for (int i = 0; i < 3; i++) {
    Serial.print(result[i], 4);  // 4 basamaklı göster
    if (i < 2) Serial.print(" ");
  }
  Serial.println("]");

  delay(5000);
}
