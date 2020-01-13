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
Lorro_BQ4050::Regt reg;

uint32_t previousMillis;
uint16_t loopInterval = 1000;

void setup() {

  Serial.begin(115200);

}

void loop() {

  uint32_t currentMillis = millis();

  if( currentMillis - previousMillis > loopInterval ){
    previousMillis = currentMillis;

    BQ4050.readReg( reg.relativeStateOfCharge );
    Serial.print( "State of charge: " );
    Serial.print( reg.relativeStateOfCharge.val );
    Serial.println( "%" );
    delay( 15 );

    BQ4050.readReg( reg.voltage );
    Serial.print( "Pack voltage: " );
    Serial.print( reg.voltage.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( reg.cellVoltage1 );
    Serial.print( "Cell voltage 1: " );
    Serial.print( reg.cellVoltage1.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( reg.cellVoltage2 );
    Serial.print( "Cell voltage 2: " );
    Serial.print( reg.cellVoltage2.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( reg.cellVoltage3 );
    Serial.print( "Cell voltage 3: " );
    Serial.print( reg.cellVoltage3.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( reg.cellVoltage4 );
    Serial.print( "Cell voltage 4: " );
    Serial.print( reg.cellVoltage4.val );
    Serial.println( "mV" );
    delay( 15 );
    
    BQ4050.readReg( reg.current );
    Serial.print( "Current: " );
    Serial.print( reg.current.val );
    Serial.println( "mA" );
    delay( 15 );

  }

}
