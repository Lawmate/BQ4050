/**
  Example of how to read battery fuel percentage, total voltage, cell voltages and current
  from TI BQ4050 fuel gauge chip.
  Fuel percentage reading will not be correct unless the battery has been calibrated for this chip
**/

//Libraries to be included
#include "Arduino.h"
#include <Lorro_BQ4050.h>

//Default address for device. Note, it is without read/write bit. When read with analyser,
//this will appear 1 bit shifted to the left
#define BQ4050addr     0x0B
//Initialise the device and library
Lorro_BQ4050 BQ4050( BQ4050addr );
//Instantiate the structs
Lorro_BQ4050::Regt regs;

uint32_t previousMillis;
uint16_t loopInterval = 1000;

void setup() {

  Serial.begin(115200);

}

void loop() {

  uint32_t currentMillis = millis();

  if( currentMillis - previousMillis > loopInterval ){
    previousMillis = currentMillis;

    BQ4050.readReg( regs.relativeStateOfCharge );
    Serial.print( "State of charge: " );
    Serial.print( regs.relativeStateOfCharge.val );
    Serial.println( "%" );
    delay( 15 );

    BQ4050.readReg( regs.voltage );
    Serial.print( "Pack voltage: " );
    Serial.print( regs.voltage.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( regs.cellVoltage1 );
    Serial.print( "Cell voltage 1: " );
    Serial.print( regs.cellVoltage1.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( regs.cellVoltage2 );
    Serial.print( "Cell voltage 2: " );
    Serial.print( regs.cellVoltage2.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( regs.cellVoltage3 );
    Serial.print( "Cell voltage 3: " );
    Serial.print( regs.cellVoltage3.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( regs.cellVoltage4 );
    Serial.print( "Cell voltage 4: " );
    Serial.print( regs.cellVoltage4.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( regs.current );
    Serial.print( "Current: " );
    Serial.print( regs.current.val );
    Serial.println( "mA" );
    delay( 15 );

  }

}
