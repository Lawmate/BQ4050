/**************************************************************************/
/*!
    @file     Arduino_BQ4050.h
    @author   Lorro


	A library for interfacing with TI BQ4050 battery fuel gauge chip


*/
/**************************************************************************/

 #include "Arduino.h"

#define BQ4050addr 														0x0B

#define VoltageReg 														0x08
#define StateOfChargeReg  										0x0E
#define DAStatus1reg  												0x71
#define CellVoltageFour  											0x3C
#define CellVoltageThree 											0x3D
#define CellVoltageTwo  											0x3E
#define CellVoltageOne  											0x3F
#define BatteryMode  													0x03
#define ManufacturerName  										0x20
#define TemperatureReg  											0x20
#define DeviceChemistryReg  									0x22
#define DFDAConfigurationReg1  								0x45 //set to 0x13 for 4 cells (default 0x12 - 3 cell)
#define DFDAConfigurationReg2  								0x7B //set to 0x13 for 4 cells (default 0x12 - 3 cell)

class Arduino_BQ4050{
 public:
	Arduino_BQ4050();
	void getDAConfiguration();
 	uint16_t getVoltage( void );
	uint16_t getTemperature();
	float getCellVoltage4();
	float getCellVoltage3();
	float getCellVoltage2();
	float getCellVoltage1();
	uint16_t getStateofCharge();
	void getBatteryMode();
	void setBatteryMode();
	void getManufacturerName();
	void getDeviceChemistry();

 private:
	byte readByteReg( char devAddress, byte regAddress );
	byte readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 );
	byte readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 );
	uint16_t read2ByteReg( char devAddress, byte regAddress );
	uint16_t readBlockReg( char devAddress, byte regAddress, byte *block );
	void writeByteReg( byte devAddress, byte regAddress, byte dataByte );
	void write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 );

};
