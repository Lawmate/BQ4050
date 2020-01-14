/**
  Example of how to write values to the flash storage on the TI BQ4050 fuel gauge chip.
  Writing to the data flash will only work if the chip is in the unsealed state.
  Most shipped products that use the chip such as smart batteries, will be in the
  sealed state.
  If you are starting with a fresh chip and want to calibrate your own battery,
  this example will be useful.
  These values are purely demonstrative. You will need to read through the docs
  at https://www.ti.com/tool/GPCCEDV
**/

//Libraries to be included
#include "Arduino.h"
#include "Lorro_BQ4050.h"

//Default address for device. Note, it is without read/write bit. When read with analyser,
//this will appear 1 bit shifted to the left
#define BQ4050addr     0x0B
//Initialise the device and library
Lorro_BQ4050 BQ4050( BQ4050addr );
//Instantiate the structs
Lorro_BQ4050::Regt registers;

void setup(){

  //In the setup function, we simply write 3 values in 3 different ways. You can either go through
  //the header file and edit the values there, or send them from here in the main program
  //The thing to be aware of is the data type. The values range from 1 to 4 bytes in length
  //and are either signed or unsigned.

  //Instantiate the data structure
  Lorro_BQ4050::DFt DataFlash;

  //This is how to write a value from the value that is stored in the header file.
  //If you edit the header file directly, use this method to write the flash from
  //your main program. This method means you don't need to know the data type in
  //the main program.
  BQ4050.writeFlash( DataFlash.protections.OCD1.threshold, DataFlash.protections.OCD1.threshold.val );

  //Put a delay of at least 15mS between writing different values, otherwise the
  //chip won't reliably respond properly
  delay(15);

  //The second method is if you write the value from the main program. You need to
  //know the data type of the value you are sending for this method.
  uint8_t delayVal = 6;
  BQ4050.writeFlash( DataFlash.protections.OCD1.delay, delayVal );
  delay(15);

  //The third method is if you don't want to look up the data type, you can use
  //the decltype function to return the datatype from the header file and cast the
  //value to be written to that type.
  BQ4050.writeFlash( DataFlash.gasGauging.state.learnedFCC, ( decltype( DataFlash.gasGauging.state.learnedFCC.val ) )2600 );

  //This is a function dedicated to writing the flash value for the number of number
  //of cells in the pack. Any value above 4 will be cut to 4.
  BQ4050.numberOfCells( 4 );

}

void loop(){

}
