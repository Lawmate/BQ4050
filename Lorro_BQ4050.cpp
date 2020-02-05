/**************************************************************************/
/*!
@file     Lorro_BQ4050.h
@author   Lorro


A library for interfacing with TI BQ4050 battery fuel gauge chip
*/
/**************************************************************************/
#include "Arduino.h"

#include <Wire.h>

#include "Lorro_BQ4050.h"

Lorro_BQ4050::Lorro_BQ4050( char addr ){

  //Initialisation function

  //Load in address
  BQ4050addr = addr;
  //Start the I2C comms
  Wire.begin();
  //Create the lookup table for doing CRC8 checks for the PEC
  CalculateTable_CRC8();

}

//Create instances of structs for use across the functions
Lorro_BQ4050::DFt dataFlash;
Lorro_BQ4050::Regt registers;

//Prints a binary number with leading zeros (Automatic Handling)
//Useful for debugging
#define PRINTBIN(Num) for (uint64_t t = (1ULL<< ((sizeof(Num)*8)-1)); t; t >>= 1) Serial.write(Num  & t ? '1' : '0');

boolean Lorro_BQ4050::getXCHGstatus(){
  if( readBlockReg( registers.operationStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.operationStatus.val[ 1 ] & 0x40 ) >> 6 ) == 1 ){
      Serial.println("Charging is disabled \t\t\t(XCHG high)");
    }else{
      Serial.println("Charging is not disabled \t\t(XCHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getXDSGstatus(){

//Function to read the register and extract the status bit for human reading

  if( readBlockReg( registers.operationStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.operationStatus.val[ 1 ] & 0x20 ) >> 5 ) == 1 ){
      Serial.println("Discharging is disabled \t(XDSG high)");
    }else{
      Serial.println("Discharging is not disabled \t\t(XDSG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getPCHGstatus(){
  if( readBlockReg( registers.operationStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.operationStatus.val[ 0 ] & 0x08 ) >> 3 ) == 1 ){
      Serial.println("Pre-charge FET is enabled (on) \t\t(PCHG high)");
    }else{
      Serial.println("Pre-charge FET is disabled (off) \t(PCHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getCHGstatus(){
  if( readBlockReg( registers.operationStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.operationStatus.val[ 0 ] & 0x04 ) >> 2 ) == 1 ){
      Serial.println("Charging FET is enabled (on) \t\t(CHG high)");
    }else{
      Serial.println("Charging FET is disabled (off) \t\t(CHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getDSGstatus(){
  if( readBlockReg( registers.operationStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.operationStatus.val[ 0 ] & 0x02 ) >> 1 ) == 1 ){
      Serial.println("Discharging FET is enabled (on) \t(DSG high)");
    }else{
      Serial.println("Discharging FET is disabled (off) \t(DSG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getCellBalancingStatus(){
  if( readBlockReg( registers.gaugingStatus ) ){
    //AND the bit with the byte then shift it right
    if( ( ( registers.gaugingStatus.val[ 0 ] & 0x10 ) >> 4 ) == 1 ){
      Serial.println("Cell balancing is enabled (on) \t\t(BAL_EN high)");
    }else{
      Serial.println("Cell balancing is disabled (off) \t(BAL_EN low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::numberOfCells( uint8_t cellNum ){
  //Catch to make sure no values too high pass
  if( cellNum > 4 ) cellNum = 4;
  //Reduce cell number value, so it starts from zero
  cellNum = cellNum - 1;
  //Mask off the end 2 bits by ANDing the byte with all ones apart from the end 2 bits
  dataFlash.settings.configuration.dAConfiguration.val = \
  ( dataFlash.settings.configuration.dAConfiguration.val & 0xFC );
  //Load the new data in by ORing the masked off value with the data value.
  dataFlash.settings.configuration.dAConfiguration.val = \
  ( dataFlash.settings.configuration.dAConfiguration.val | cellNum );
  //Use the writeFlash function to write the new number of cells value
  Serial.print( "DA config byte from cpp: " );
  Serial.println( dataFlash.settings.configuration.dAConfiguration.val, HEX );
  // writeFlash( DF.settings.configuration.dAConfiguration, DF.settings.configuration.dAConfiguration.val );
  return true;
}

void Lorro_BQ4050::FETtoggle(){
  writeCommand( BQ4050addr, FETcontrol );
}

void Lorro_BQ4050::deviceReset(){
  writeCommand( BQ4050addr, deviceResetReg );
}

// I2C functions below here
//----------------------------------------------


boolean Lorro_BQ4050::readDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen ){

  //Function to read data from the device registers

  //Add in the device address to the buffer
  Wire.beginTransmission( devAddress );
  //Add the one byte register address
  Wire.write( regAddress );
  //Send out buffer and log response
  byte ack = Wire.endTransmission();
  //If data is ackowledged, proceed
  if( ack == 0 ){
    //Request a number of bytes from the device address
    Wire.requestFrom( devAddress , (int16_t) arrLen );
    //If there is data in the in buffer
    if( Wire.available() > 0 ){
      //Cycle through, loading data into array
      for( uint8_t i = 0; i < arrLen; i++ ){
        dataVal[i] = Wire.receive();
      }
    }
    return true;
  }else{
    return false; //if I2C comm fails
  }

}

boolean Lorro_BQ4050::writeDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen ){

  //Function that writes data to a register with a one byte address

  //Create an array of all byte to be sent to enable PEC calculation later
  //Device address is shifted right 1 bit to simulate a write bit
  byte byteArr[ 6 ] = { ( byte )( devAddress << 1 ), regAddress, 0, 0, 0, 0 };
  //Load address into buffer
  Wire.beginTransmission( devAddress );
  //The register addres we're writing to
  Wire.write( regAddress );
  //Cycle through the byte array
  for( int i = 0; i < arrLen; i++ ){
    //Add the data after the device address and the register address
    byteArr[ i + 2 ] = dataVal [ i ];
    //Send the data to the buffer
    Wire.write( dataVal[ i ] );
  }
  //Calculate the PEC byte based on the other bytes being sent
  byte PECcheck = Compute_CRC8( byteArr, arrLen + 2 );
  //Add PEC byte to buffer
  Wire.write( PECcheck );
  //Send out buffer, logging whether an ack is received
  byte ack = Wire.endTransmission();
  //return result of ack
  if( ack == 0 ){
    return true;
  }else{
    return false; //if I2C comm fails
  }

}

boolean Lorro_BQ4050::writeDFByteReg( char devAddress, int16_t regAddress, byte *data, uint8_t arrSize ){

  //function that handles writing data to the flash on the BQ4050.
  //Data is between 1 and 4 bytes long
  //Addresses are 2 bytes long

  //For CRC calculation, the address needs to include the write bit, so shifts to the left
  byte addr = devAddress << 1;
  //This is a value that gets sent to the BQ4050 to let it know how many bytes are heading its way
  byte blockLen = arrSize + 2;
  //Desconstructs the 16 bit int into 2 bytes
  byte regAddress1 = byte( regAddress );
  byte regAddress2 = byte( regAddress >> 8 );
  //for simplicity, we create an array to hold all the byte we will be sending
  byte byteArr[37] = { addr, 0x44, blockLen, regAddress1, regAddress2, data[0], data[1], data[2], data[3] };
  //This array is then sent to the CRC checker. The CRC includes all bytes sent.
  //The arrSize var is only the data, so regardless of its size, there are another 5 bytes to be sent
  byte PECcheck = Compute_CRC8( byteArr, arrSize + 5 );

  //Send the bytes out
  Wire.beginTransmission( devAddress );
  Wire.send( 0x44 );
  Wire.send( arrSize + 2 );
  //The address is little endian, so LSB is sent first
  Wire.send( regAddress1 );
  Wire.send( regAddress2 );
  //loop for the number of data bytes
  for( int i = 0; i < arrSize; i++ ){
    Wire.send( data[i] );
  }
  //send CRC at the end
  Wire.send( PECcheck );
  byte ack = Wire.endTransmission();
  if( ack == 0 ){
    return true;
  }else{
    return false; //if I2C comm fails
  }

}

boolean Lorro_BQ4050::writeCommand( char devAddress, byte regAddress ){

  //Function that simply sends out a command to the device byt way of writing
  //a zero to a particular address

  Wire.beginTransmission( devAddress );
  //MAC access
  Wire.write( 0x44 );
  //little endian of address
  Wire.write( regAddress );
  Wire.write( 0x00 );
  byte ack = Wire.endTransmission();
  if( ack == 0 ){
    return true;
  }else{
    return false; //if I2C comm fails
  }

}

byte crctable[256];
boolean printResults = false;

void Lorro_BQ4050::CalculateTable_CRC8(){

//Function that generates byte array as a lookup table to quickly create a CRC8 for the PEC

  const byte generator = 0x07;
  /* iterate over all byte values 0 - 255 */
  for (int divident = 0; divident < 256; divident++){
    byte currByte = (byte)divident;
    /* calculate the CRC-8 value for current byte */
    for (byte bit = 0; bit < 8; bit++){
      if ((currByte & 0x80) != 0){
        currByte <<= 1;
        currByte ^= generator;
      }
      else{
        currByte <<= 1;
      }
    }
    /* store CRC value in lookup table */
    crctable[divident] = currByte;
    if( printResults ){
      if( divident % 16 == 0 && divident > 2 ){
        Serial.println();
      }
      if( currByte < 16 ) Serial.print( "0" );
      Serial.print( currByte, HEX );
      Serial.print( "\t" );
    }
  }
  if( printResults ){
    Serial.println();
  }
}

byte Lorro_BQ4050::Compute_CRC8(byte *bytes, uint8_t byteLen){

  //Function to check byte array to be sent out against lookup table to efficiently calculate PEC

  byte crc = 0;
  for( int i = 0; i < byteLen; i++ ){
    /* XOR-in next input byte */
    byte data = (byte)(bytes[i] ^ crc);
    /* get current CRC value = remainder */
    crc = (byte)(crctable[data]);
    if( printResults ){
      Serial.print( "byte value: " );
      Serial.print( bytes[i], HEX );
      Serial.print( "\tlookup position: " );
      Serial.print( data, HEX );
      Serial.print( "\tlookup value: " );
      Serial.println( crc, HEX );
    }
  }

  return crc;
}
