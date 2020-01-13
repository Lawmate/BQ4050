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
Lorro_BQ4050::DFt DF;
Lorro_BQ4050::Regt reg;

// Prints a binary number with leading zeros (Automatic Handling)
#define PRINTBIN(Num) for (uint64_t t = (1ULL<< ((sizeof(Num)*8)-1)); t; t >>= 1) Serial.write(Num  & t ? '1' : '0');

boolean Lorro_BQ4050::getXCHGstatus(){
  if( readBlockReg( reg.operationStatus ) ){
    if( ( ( reg.operationStatus.val[ 2 ] & 0x40 ) >> 6 ) == 1 ){ //AND the bit with the byte then shift it right
      Serial.println("Charging is disabled \t(XCHG high)");
    }else{
      Serial.println("Charging is not disabled \t(XCHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getXDSGstatus(){
  if( readBlockReg( reg.operationStatus ) ){
    if( ( ( reg.operationStatus.val[2] & 0x20 ) >> 5 ) == 1 ){ //AND the bit with the byte then shift it right
      Serial.println("Discharging is disabled \t(XDSG high)");
    }else{
      Serial.println("Discharging is not disabled \t(XDSG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getPCHGstatus(){
  if( readBlockReg( reg.operationStatus ) ){
    if( ( ( reg.operationStatus.val[1] & 0x08 ) >> 3 ) == 1 ){ //AND the bit with the byte then shift it right
      Serial.println("Pre-charge FET is enabled (on) \t(PCHG high)");
    }else{
      Serial.println("Pre-charge FET is disabled (off) \t(PCHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getCHGstatus(){
  if( readBlockReg( reg.operationStatus ) ){
    if( ( ( reg.operationStatus.val[1] & 0x04 ) >> 2 ) == 1 ){ //AND the bit with the byte then shift it right
      Serial.println("Charging FET is enabled (on) \t(CHG high)");
    }else{
      Serial.println("Charging FET is disabled (off) \t(CHG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

boolean Lorro_BQ4050::getDSGstatus(){
  if( readBlockReg( reg.operationStatus ) ){
    if( ( ( reg.operationStatus.val[1] & 0x02 ) >> 1 ) == 1 ){ //AND the bit with the byte then shift it right
      Serial.println("Discharging FET is enabled (on) \t(DSG high)");
    }else{
      Serial.println("Discharging FET is disabled (off) \t(DSG low)");
    }
    return true; //return if the bit read has been successful
  }else{
    return false;
  }
}

void Lorro_BQ4050::FETtoggle(){
  writeCommand( BQ4050addr, FETcontrol );
}

void Lorro_BQ4050::deviceReset(){
  byte manu = 0x44;
  write2ByteReg( BQ4050addr, manu, deviceResetReg, 0x00 );
}
//
// template<typename T, typename S>
// void Lorro_BQ4050::writeFlash( const T& dataParam, const S datVal){
//
//   constexpr uint8_t byteLen = sizeof( datVal );
//   byte valBytes[ byteLen ];
//   for( int i = 0; i < byteLen; i++ ){
//     valBytes[ i ] = datVal >> ( i * 8 );
//   }
//   writeDFByteReg( BQ4050addr, dataParam.addr, valBytes, byteLen );
//
// }

// void Lorro_BQ4050::writeThreshold( int16_t datVal ){
//   DFt DF;
//   writeFlash( DF.protections.OCD1.threshold, datVal );
// }

// void Lorro_BQ4050::writeProtectionsOCD1Delay( uint8_t datVal ){
//   DFt DF;
//   writeFlash( DF.protections.OCD1.delay, datVal );
// }

// void Lorro_BQ4050::writeOCD1Threshold(){
//   // byte data[ 4 ];
//   // uint8_t datLen = sizeof( DF.Protections.OCD1.Threshold.val );
//   // for( uint8_t i = 0; i < datLen; i++ ){
//   //   data[i] = byte( DF.Protections.OCD1.Threshold.val >> ( i * 8 ) );
//   // }
//   // writeDFByteReg( BQ4050addr, DF.Protections.OCD1.Threshold.addr, data, datLen );
// }

// boolean Lorro_BQ4050::getDABlock(){
//   byte addr1 = 0x45;
//   byte addr2 = 0x60;
//   byte blockVals[32];
//   if( readDFBlockReg(  BQ4050addr, addr1, addr2, blockVals ) ){
//     for( int i = 0; i < 32; i++ ){
//       Serial.println( blockVals[i], HEX );
//     }
//     return true;
//   } else{
//     return false;
//   }
// }

// I2C functions below here
//----------------------------------------------



// uint16_t Lorro_BQ4050::read2ByteReg(char devAddress, byte regAddress){
//
//   byte dataByte[2] = {0};
//   Wire.beginTransmission( devAddress );
//   Wire.write( regAddress );
//   Wire.endTransmission();
//
//   Wire.requestFrom( devAddress , 3);
//   if( Wire.available() > 0 ){
//     dataByte[0] = Wire.receive();
//     dataByte[1] = Wire.receive();
//   }
//   uint16_t val = ( dataByte[1] << 8 ) + dataByte[0];
//   return val;
//
// }

boolean Lorro_BQ4050::readDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  byte ack = Wire.endTransmission();
  if( ack == 0 ){
    Wire.requestFrom( devAddress , (int16_t) arrLen );
    if( Wire.available() > 0 ){
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

  byte byteArr[ 6 ] = { ( byte )( devAddress << 1 ), regAddress, 0, 0, 0, 0 };
  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  for( int i = 0; i < arrLen; i++ ){
    byteArr[ i + 2 ] = dataVal [ i ];
    Wire.write( dataVal[ i ] );
  }
  byte PECcheck = Compute_CRC8( byteArr, arrLen + 2 );
  Wire.write( PECcheck );
  byte ack = Wire.endTransmission();
  if( ack == 0 ){
    return true;
  }else{
    return false; //if I2C comm fails
  }

}

// byte Lorro_BQ4050::readByteReg( char devAddress, byte regAddress ){
//   byte dataByte = 0;
//
//   Wire.beginTransmission( devAddress );
//   Wire.write( regAddress );
//   Wire.endTransmission();
//
//   Wire.requestFrom( devAddress , 1);
//   if( Wire.available() > 0 ){
//     dataByte = Wire.receive();
//   }
//   return dataByte;
// }

// byte Lorro_BQ4050::readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 ){
//
//   byte dataByte = 0;
//   byte sentData[3] = { regAddress2, regAddress1 };
//
//   Wire.beginTransmission( devAddress );
//   Wire.send( 0x00 );
//   Wire.send( sentData, 2 );
//   Serial.print("return status: ");
//   Serial.println(Wire.endTransmission());
//   delay(5);
//
//   Wire.requestFrom( devAddress , 3);
//   if( Wire.available() > 0 ){
//     dataByte = Wire.receive();
//   }
//
//   return dataByte;
// }

// boolean Lorro_BQ4050::readDFBlockReg( char devAddress, byte regAddress1, byte regAddress2, byte *blockData ){
//
//   byte DFblockReg[32];
//
//   Wire.beginTransmission( devAddress );
//   Wire.send( 0x44 );
//   Wire.send( 0x02 );
//   Wire.send( regAddress2 );
//   Wire.send( regAddress1 );
//   Wire.send( regAddress1 );
//   Wire.endTransmission();
//
//   delay(5);
//
//   if( readBlockReg( BQ4050addr, 0x44, DFblockReg ) ){
//     for( int i = 0; i < 32; i++ ){
//       blockData[i] = DFblockReg[i];
//     }
//     return true;
//   }else{
//     return false;
//   }
//
// }

void Lorro_BQ4050::writeDFByteReg( char devAddress, int16_t regAddress, byte *data, uint8_t arrSize ){

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
  Wire.endTransmission();
  delay(20);

}

// byte Lorro_BQ4050::readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 ){
//
//   byte dataByte = 0;
// //   byte sentData[3] = { regAddress2, regAddress1 };
// //
// //   Wire2.i2c_start( B0010110 );
// //   Wire2.i2c_write( 0x44 );
// //   Wire2.i2c_write( sentData[0] );
// //   Wire2.i2c_write( sentData[1] );
// //   Serial.print("return status: ");
// //   Wire2.i2c_stop();
// // //  Serial.println(Wire.endTransmission());
// //   delay(5);
// //
// //   Wire2.i2c_start( B0010111 );
// // //  Wire2.i2c_write( 0x44 );
// //   dataByte = Wire2.i2c_read(false);
// //   Wire2.i2c_read(false);
// //   Wire2.i2c_read(false);
// //   Wire2.i2c_read(false);
// //   Wire2.i2c_read(false);
// //   Wire2.i2c_stop();
// //  Wire.write( 0x00 );
// //  if( Wire.available() > 0 ){
// //    dataByte = Wire.receive();
// //  }
//
//   return dataByte;
// }

// byte Lorro_BQ4050::readBlockReg( char devAddress, byte regAddress, byte *block ){
//
//   Wire.beginTransmission( devAddress );
//   Wire.write( regAddress );
//   byte ack = Wire.endTransmission();
//   //successful ack will equal 0. Failed ack will be other number
//
//   if( ack == 0 ){
//     Wire.requestFrom( devAddress , 32);
//     if( Wire.available() > 0 ){
//       for(int i = 0; i < 32; i++ ){
//         block[i] = Wire.receive();
//       }
//     }
//     return true;
//   }else{
//     //if no response from chip, write 0's to array
//     for(int i = 0; i < 32; i++ ){
//       block[i] = '0';
//     }
//     return false; //if I2C comm fails
//   }
//
// }

// void Lorro_BQ4050::writeByteReg( byte devAddress, byte regAddress, byte dataByte ){
//
//   Wire.beginTransmission( devAddress );
//   Wire.write( regAddress );
//   Wire.write( dataByte );
//   Wire.endTransmission();
//
// }
//
// void Lorro_BQ4050::write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 ){
//
//   Wire.beginTransmission( devAddress );
//   Wire.write( regAddress );
//   Wire.write( dataByte1 );
//   Wire.write( dataByte2 );
//   Wire.endTransmission();
//
// }
//
// void Lorro_BQ4050::readMACblock( char devAddress, byte regAddress, byte *block ){
//
//     Wire.beginTransmission( devAddress );
//     Wire.write( 0x44 ); //MAC access
//     // Wire.write( 0x02 ); //for the following 2 bytes
//     Wire.write( regAddress ); //little endian of address
//     Wire.write( 0x00 );
//     Wire.endTransmission();
//
//     // Wire.beginTransmission( devAddress );
//     // Wire.write( 0x44 );
//
//     Wire.requestFrom( devAddress , 34);
//     Wire.write( 0x44 );
//     // Wire.write( 0x20 ); //write the number of bytes in the block (32)
//     if( Wire.available() > 0 ){
//       for(int i = 0; i < 32; i++ ){
//         block[i] = Wire.receive();
//       }
//     }
//     // return *block;
// }


void Lorro_BQ4050::writeCommand( char devAddress, byte regAddress ){

    Wire.beginTransmission( devAddress );
    Wire.write( 0x44 ); //MAC access
    Wire.write( regAddress ); //little endian of address
    Wire.write( 0x00 );
    Wire.endTransmission();

}

byte crctable[256];
boolean printResults = false;

void Lorro_BQ4050::CalculateTable_CRC8()
{
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

byte Lorro_BQ4050::Compute_CRC8(byte *bytes, uint8_t byteLen)
{
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
