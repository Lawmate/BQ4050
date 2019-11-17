/**************************************************************************/
/*!
@file     Arduino_BQ4050.h
@author   Lorro


A library for interfacing with TI BQ4050 battery fuel gauge chip
*/
/**************************************************************************/
#include "Arduino.h"

#include <Wire.h>

#include "Arduino_BQ4050.h"

void Arduino_BQ4050::getDAConfiguration(){
  byte DAConfig = readDFByteReg( BQ4050addr, DFDAConfigurationReg1, DFDAConfigurationReg2 );
//  printByteVal( Voltage );
//  Serial.print("address byte");
//  Serial.println( BQ4050addr << 1, HEX );
  Serial.print("DAConfig: ");
  // PRINTBIN(DAConfig);
//  Serial.print(Voltage / 1000);
  Serial.print(", hex: ");
  Serial.println(DAConfig, HEX);
}

uint16_t Arduino_BQ4050::getVoltage( void ){
  uint16_t Voltage = read2ByteReg( BQ4050addr, VoltageReg );
//  printByteVal( Voltage );
//  Serial.print("address byte");
//  Serial.println( BQ4050addr << 1, HEX );
  Serial.print("Battery voltage: ");
  Serial.print(Voltage / 1000);
  Serial.println("V");
  return Voltage;
}

uint16_t Arduino_BQ4050::getTemperature(){
//  float Temperature = float( read2ByteReg( BQ4050addr, TemperatureReg ) );
  uint16_t Temperature = read2ByteReg( BQ4050addr, TemperatureReg );
  Serial.print("Temperature: ");
//  Serial.print( ( Temperature / 10 ) - 273.1 );
  Serial.print( Temperature );
  Serial.println("C");
  return Temperature;
}

float Arduino_BQ4050::getCellVoltage4(){
 float CellVoltage4 = float( read2ByteReg( BQ4050addr, CellVoltageFour ) ) / 1000;
  Serial.print("Cell 4 voltage: ");
  Serial.print(CellVoltage4);
  Serial.println("V");
  return CellVoltage4;
}

float Arduino_BQ4050::getCellVoltage3(){
 float CellVoltage3 = float( read2ByteReg( BQ4050addr, CellVoltageThree ) ) / 1000;
  Serial.print("Cell 3 voltage: ");
  Serial.print(CellVoltage3 );
  Serial.println("V");
  return CellVoltage3;
}

float Arduino_BQ4050::getCellVoltage2(){
 float CellVoltage2 = float( read2ByteReg( BQ4050addr, CellVoltageTwo ) ) / 1000;
  Serial.print("Cell 2 voltage: ");
  Serial.print(CellVoltage2 );
  Serial.println("V");
  return CellVoltage2;
}

float Arduino_BQ4050::getCellVoltage1(){
 float CellVoltage1 = float( read2ByteReg( BQ4050addr, CellVoltageOne ) ) / 1000;
  Serial.print("Cell 1 voltage: ");
  Serial.print(CellVoltage1 );
  Serial.println("V");
  return CellVoltage1;
}

uint16_t Arduino_BQ4050::getStateofCharge(){
  uint16_t stateOfCharge = read2ByteReg( BQ4050addr, StateOfChargeReg );
  Serial.print("Battery state of charge: ");
  Serial.print(stateOfCharge);
  Serial.println("%");
  return stateOfCharge;
}

void Arduino_BQ4050::getBatteryMode(){
  // uint16_t BatteryModeVal = read2ByteReg( BQ4050addr, BatteryMode );
  Serial.print("Battery mode: ");
  // PRINTBIN(BatteryModeVal);
  Serial.println();
}

void Arduino_BQ4050::setBatteryMode(){
  // uint16_t BatteryModeVal = read2ByteReg( BQ4050addr, BatteryMode );
  Serial.print("Battery mode: ");
  // PRINTBIN(BatteryModeVal);
  Serial.println();
}

// void Arduino_BQ4050::DAStatus1(){
//   // byte DAStatusData[32];
//   // readBlockReg( BQ4050addr, DAStatus1reg, DAStatusData );
//   // for(int i = 0; i < 32; i++ ){
//   //   printByteVal( DAStatusData[i] );
//   // }
//   // for(int i = 0; i < 16; i++ ){
//   //   uint16_t staData = DAStatusData[ ( i * 2 ) + 1 ] << 8 | DAStatusData[ i * 2 ];
//   //   Serial.println( staData );
//   // }
// }

void Arduino_BQ4050::getManufacturerName(){
  byte ManufacturerNameData[32];
  readBlockReg( BQ4050addr, ManufacturerName, ManufacturerNameData );
  Serial.print("Manufacturer name: ");
  for( int i = 1; i < 18; i++ ){
    Serial.print( char( ManufacturerNameData[i] ) );
  }
  Serial.println();
}

void Arduino_BQ4050::getDeviceChemistry(){
  byte DeviceChemistryData[32];
  readBlockReg( BQ4050addr, DeviceChemistryReg, DeviceChemistryData );
  Serial.print("Device chemistry: ");
  for( int i = 1; i < 5; i++ ){
    Serial.print( char( DeviceChemistryData[i] ) );
  }
  Serial.println();
}

uint16_t Arduino_BQ4050::read2ByteReg(char devAddress, byte regAddress){

  byte dataByte[2] = {0};
  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 3);
  if( Wire.available() > 0 ){
    dataByte[0] = Wire.receive();
    dataByte[1] = Wire.receive();
  }
  uint16_t val = ( dataByte[1] << 8 ) + dataByte[0];
  return val;

}

byte Arduino_BQ4050::readByteReg( char devAddress, byte regAddress ){
  byte dataByte = 0;

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 1);
  if( Wire.available() > 0 ){
    dataByte = Wire.receive();
  }
  return dataByte;
}

byte Arduino_BQ4050::readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 ){

  byte dataByte = 0;
  byte sentData[3] = { regAddress2, regAddress1 };

  Wire.beginTransmission( devAddress );
  Wire.send( 0x00 );
  Wire.send( sentData, 2 );
  Serial.print("return status: ");
  Serial.println(Wire.endTransmission());
  delay(5);

  Wire.requestFrom( devAddress , 3);
  if( Wire.available() > 0 ){
    dataByte = Wire.receive();
  }

  return dataByte;
}

byte Arduino_BQ4050::readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 ){

  byte dataByte = 0;
//   byte sentData[3] = { regAddress2, regAddress1 };
//
//   Wire2.i2c_start( B0010110 );
//   Wire2.i2c_write( 0x44 );
//   Wire2.i2c_write( sentData[0] );
//   Wire2.i2c_write( sentData[1] );
//   Serial.print("return status: ");
//   Wire2.i2c_stop();
// //  Serial.println(Wire.endTransmission());
//   delay(5);
//
//   Wire2.i2c_start( B0010111 );
// //  Wire2.i2c_write( 0x44 );
//   dataByte = Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_stop();
//  Wire.write( 0x00 );
//  if( Wire.available() > 0 ){
//    dataByte = Wire.receive();
//  }

  return dataByte;
}

uint16_t Arduino_BQ4050::readBlockReg( char devAddress, byte regAddress, byte *block ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 34);
  Wire.write( 0x20 ); //write the number of bytes in the block (32)
  if( Wire.available() > 0 ){
    for(int i = 0; i < 32; i++ ){
      block[i] = Wire.receive();
    }
  }
  return *block;
}

void Arduino_BQ4050::writeByteReg( byte devAddress, byte regAddress, byte dataByte ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.write( dataByte );
  Wire.endTransmission();

}

void Arduino_BQ4050::write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.write( dataByte1 );
  Wire.write( dataByte2 );
  Wire.endTransmission();

}
