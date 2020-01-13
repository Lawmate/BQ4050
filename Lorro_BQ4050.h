/**************************************************************************/
/*!
    @file     Lorro_BQ4050.h
    @author   Lorro


	A library for interfacing with TI BQ4050 battery fuel gauge chip
  Includes template function in header file


*/
/**************************************************************************/

 #include "Arduino.h"

#define AtRateReg                             0x04
#define AtRateTimeToFullReg                   0x05
#define AtRateTimetoEmptyReg                  0x06
#define AtRateOKReg                           0x07
#define VoltageReg 														0x08
#define CurrentReg 														0x0A
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
#define operationStatusReg                    0x54
#define FETcontrol                            0x22
#define deviceResetReg                        0x41

class Lorro_BQ4050{
 public:
	Lorro_BQ4050( char addr );
  void init();
 	// Lorro_BQ4050(  );
	void getDAConfiguration();
 	uint16_t getVoltage( void );
  boolean getCurrent( int16_t *currentVal );
  boolean getAtRate( int16_t *currentVal );
  boolean setAtRate( int16_t *currentVal );
  boolean getAtRateTimeToFull( uint16_t *mins );
  boolean getAtRateTimeToEmpty( uint16_t *mins );
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
  boolean getOperationStatus(byte *opStatArr);
  boolean getXCHGstatus();
  boolean getXDSGstatus();
  boolean getPCHGstatus();
  boolean getCHGstatus();
  boolean getDSGstatus();
  void FETtoggle();
  void deviceReset();
  template<typename T, typename S>
  boolean writeFlash( const T& dataParam, const S datVal){

    //Function for writing to the data Flash register.
    //The first variable to come in is the struct and the second is the data.
    //This could be the dataParam.val value or an arbitrary value of correct type.

    //Calculate length of data
    constexpr uint8_t byteLen = sizeof( datVal );
    //Create an array to hold the returned data
    byte valBytes[ byteLen ];
    //Loop through array
    for( int i = 0; i < byteLen; i++ ){
      //Shift the data out to the individual bytes, 8 bits at a time.
      valBytes[ i ] = datVal >> ( i * 8 );
    }
    //Run function to write bytes into the register.
    if( writeDFByteReg( BQ4050addr, dataParam.addr, valBytes, byteLen ) ){
      return true;
    }else{
      return false;
    }

  }
  template<typename T, typename S>
  boolean writeReg( const T& dataParam, const S datVal){

    //Function for writing to the data register.
    //The first variable to come in is the struct and the second is the data.
    //This could be the dataParam.val value or an arbitrary value of correct type.

    //Calculate length of data
    constexpr uint8_t byteLen = sizeof( datVal );
    //Create an array to hold the returned data
    byte valBytes[ byteLen ];
    //Loop through array
    for( int i = 0; i < byteLen; i++ ){
      //Shift the data out to the individual bytes, 8 bits at a time.
      valBytes[ i ] = datVal >> ( i * 8 );
    }
    //Run function to write bytes into the register.
    if( writeDataReg( BQ4050addr, ( byte )dataParam.addr, valBytes, byteLen ) ){
      return true;
    }else{
      return false;
    }

  }
  template<typename T>
  boolean readReg( T& dataParam ){

    //This is a function for reading data words.
    //The number of bytes that make up a word is either 1 or 2.

    //Measure the number of bytes to look for
    constexpr uint8_t byteLen = sizeof( dataParam.val );
    //Create an array to hold the returned data
    byte valBytes[ byteLen ];
    //Function to handle the I2C comms.
    if( readDataReg( BQ4050addr, ( byte )dataParam.addr, valBytes, byteLen ) ){
      //Cycle through array of data
      for( int i = 0; i < byteLen; i++ ){
        //Shift each byte in to the right, in steps of 8 bits. The resulting data is type cast, by getting the type with decltype
        dataParam.val = ( decltype( dataParam.val ) ) ( dataParam.val | ( valBytes[ i ] << ( 8 * i ) ) );
      }
      return true;
    }else{
      return false;
    }

  }
  template<typename T>
  boolean readBlockReg( T& dataParam ){

    //This is a function for reading data blocks.
    //Data blocks are returned with the first byte denoting how many data bytes are to follow
    //so ignore the value of the first byte as it is not data.
    //Function must be passed an array of bytes.

    //Work out how many bytes to request, plus 1 for number of bytes to follow byte
    constexpr uint8_t byteLen = sizeof( dataParam.val ) + 1;
    //Create a byte array to store the returned data
    byte valBytes[ byteLen ];
    //Send the read function the correct data
    if( readDataReg( BQ4050addr, ( byte )dataParam.addr, valBytes, byteLen ) ){
      //cycle through the array, one less than the length, to ignore the first byte
      for( int i = 0; i < ( byteLen - 1 ); i++ ){
        //Skip over the first byte of the read array
        dataParam.val[ i ] = valBytes[ i + 1 ];
      }
      return true;
    }else{
      return false;
    }

  }
  void writeThreshold( int16_t datVal );
  void writeProtectionsOCD1Delay( uint8_t datVal );
  void writeOCD1Threshold();
  char BQ4050addr;
  boolean getDABlock();

  String operationStatusBits[32] = {  "",
                                      "",
                                      "Emergency shutdown (active)",
                                      "Cell balancing (active)",
                                      "CC in sleep mode (active)",
                                      "ADC in sleep mode (active)",
                                      "CC calibration",
                                      "Initialisation after reset (active)",
                                      "Sleep mode triggered (active)",
                                      "400khz SMBus mode (active)",
                                      "Calibration offset",
                                      "Calibration output",
                                      "Auto calibrate CC",
                                      "Authentication in progress",
                                      "LED display on",
                                      "Shutdown triggered by command",
                                      "Sleep mode conditions met",
                                      "XCHG charging disabled",
                                      "XDSG discharging disabled",
                                      "Permanent failiure made",
                                      "Safety mode (active)",
                                      "Shutdown triggered by low pack voltage",
                                      "Security mode",
                                      "Security mode",
                                      "Battery trip point",
                                      "Smoothing active status",
                                      "fuse status",
                                      "",
                                      "Precharge FET status",
                                      "Charge FET status",
                                      "Discharge FET status",
                                      "System present low"};
    struct Regt{
      struct RemainingCapAlarmt{
        uint16_t val = 300; //mAh
        uint8_t addr = 0x01;
      } remainingCapAlarm;
      struct RemainingTimeAlarmt{
        uint16_t val = 10; //min
        uint8_t addr = 0x02;
      } remainingTimeAlarm;
      struct BatteryModet{
        uint16_t val = 0;
        uint8_t addr = 0x03;
      } batteryMode;
      struct AtRatet{
        int16_t val = 0;
        uint8_t addr = 0x04;
      } atRate;
      struct AtRateTimeToFullt{
        uint16_t val = 0;
        uint8_t addr = 0x05;
      } atRateTimeToFull;
      struct AtRateTimeToEmptyt{
        uint16_t val = 0;
        uint8_t addr = 0x06;
      } atRateTimeToEmpty;
      struct AtRateOKt{
        uint16_t val = 0;
        uint8_t addr = 0x07;
      } atRateOK;
      struct Temperaturet{
        uint16_t val = 0;
        uint8_t addr = 0x08;
      } temperature;
      struct Voltaget{
        uint16_t val = 0;
        uint8_t addr = 0x09;
      } voltage;
      struct Currentt{
        int16_t val = 0;
        uint8_t addr = 0x0A;
      } current;
      struct AverageCurrentt{
        int16_t val = 0;
        uint8_t addr = 0x0B;
      } averageCurrent;
      struct MaxErrort{
        uint8_t val = 0;
        uint8_t addr = 0x0C;
      } maxError;
      struct RelativeStateOfCharget{
        uint8_t val = 0;
        uint8_t addr = 0x0D;
      } relativeStateOfCharge;
      struct AbsoluteStateOfCharget{
        uint8_t val = 0;
        uint8_t addr = 0x0E;
      } absoluteStateOfCharge;
      struct RemainingCapacityt{
        uint16_t val = 0;
        uint8_t addr = 0x0F;
      } remainingCapacity;
      struct FullChargeCapacityt{
        uint16_t val = 0;
        uint8_t addr = 0x10;
      } fullChargeCapacity;
      struct RunTimeToEmptyt{
        uint16_t val = 0;
        uint8_t addr = 0x11;
      } runTimeToEmpty;
      struct AverageTimeToEmptyt{
        uint16_t val = 0;
        uint8_t addr = 0x12;
      } averageTimeToEmpty;
      struct AverageTimeToFullt{
        uint16_t val = 0;
        uint8_t addr = 0x13;
      } averageTimeToFull;
      struct ChargingCurrentt{
        uint16_t val = 0;
        uint8_t addr = 0x14;
      } chargingCurrent;
      struct ChargingVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x15;
      } chargingVoltage;
      struct BatteryStatust{
        uint16_t val = 0;
        uint8_t addr = 0x16;
      } batteryStatus;
      struct CycleCountt{
        uint16_t val = 0;
        uint8_t addr = 0x17;
      } cycleCount;
      struct DesignCapacityt{
        uint16_t val = 0;
        uint8_t addr = 0x18;
      } designCapacity;
      struct DesignVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x19;
      } designVoltage;
      struct Specificationt{
        uint16_t val = 0;
        uint8_t addr = 0x1A;
      } specification;
      struct ManufacturerDatat{
        uint16_t val = 0;
        uint8_t addr = 0x1B;
      } manufacturerData;
      struct SerialNumbert{
        uint16_t val = 0;
        uint8_t addr = 0x1C;
      } serialNumber;
      struct ManufacturerNamet{
        byte val[ 18 ];
        uint8_t addr = 0x20;
      } manufacturerName;
      struct DeviceNamet{
        byte val[ 7 ];
        uint8_t addr = 0x21;
      } deviceName;
      struct DeviceChemistryt{
        byte val[ 5 ];
        uint8_t addr = 0x22;
      } deviceChemistry;
      struct CellVoltage4t{
        uint16_t val = 0;
        uint8_t addr = 0x3C;
      } cellVoltage4;
      struct CellVoltage3t{
        uint16_t val = 0;
        uint8_t addr = 0x3D;
      } cellVoltage3;
      struct CellVoltage2t{
        uint16_t val = 0;
        uint8_t addr = 0x3E;
      } cellVoltage2;
      struct CellVoltage1t{
        uint16_t val = 0;
        uint8_t addr = 0x3F;
      } cellVoltage1;
      struct BTPDischargeSett{
        uint16_t val = 0;
        uint8_t addr = 0x4A;
      } bTPDischargeSet;
      struct BTPChargeSett{
        uint16_t val = 0;
        uint8_t addr = 0x4B;
      } bTPChargeSet;
      struct StateOfHealtht{
        uint16_t val = 0;
        uint8_t addr = 0x4F;
      } stateOfHealth;
      struct SafetyAlertt{
        byte val[ 5 ];
        uint8_t addr = 0x50;
      } safetyAlert;
      struct SafetyStatust{
        byte val[ 5 ];
        uint8_t addr = 0x51;
      } safetyStatus;
      struct PFAlertt{
        byte val[ 5 ];
        uint8_t addr = 0x52;
      } pFAlert;
      struct PFStatust{
        byte val[ 5 ];
        uint8_t addr = 0x53;
      } pFStatus;
      struct OperationStatust{
        byte val[ 5 ];
        uint8_t addr = 0x54;
      } operationStatus;
      struct ChargingStatust{
        byte val[ 5 ];
        uint8_t addr = 0x55;
      } chargingStatus;
      struct GaugingStatust{
        byte val[ 5 ];
        uint8_t addr = 0x56;
      } gaugingStatus;
      struct ManufacturingStatust{
        byte val[ 5 ];
        uint8_t addr = 0x57;
      } manufacturingStatus;
      struct AFERegistert{
        byte val[ 21 ];
        uint8_t addr = 0x58;
      } aFERegister;
      struct LifetimeDataBlock1t{
        byte val[ 32 ];
        uint8_t addr = 0x60;
      } lifetimeDataBlock1;
      struct LifetimeDataBlock2t{
        byte val[ 8 ];
        uint8_t addr = 0x61;
      } lifetimeDataBlock2;
      struct LifetimeDataBlock3t{
        byte val[ 16 ];
        uint8_t addr = 0x62;
      } lifetimeDataBlock3;
      struct LifetimeDataBlock4t{
        byte val[ 32 ];
        uint8_t addr = 0x63;
      } lifetimeDataBlock4;
      struct LifetimeDataBlock5t{
        byte val[ 20 ];
        uint8_t addr = 0x64;
      } lifetimeDataBlock5;
      struct ManufacturerInfot{
        byte val[ 32 ];
        uint8_t addr = 0x70;
      } manufacturerInfo;
      struct DAStatust1t{
        byte val[ 32 ];
        uint8_t addr = 0x71;
      } dAStatust1;
      struct DAStatust2t{
        byte val[ 14 ];
        uint8_t addr = 0x72;
      } dAStatust2;
    } ;
    struct DFt{
      struct Protectionst{
        struct CUVt{ //Cell undervoltage settings
          struct Thresholdt{
            int16_t val = 2850; //milliVolts
            uint16_t addr = 0x4481;
          } threshold;
          struct Delayt{
            uint8_t val = 2; //seconds
            uint16_t addr = 0x4483;
          } delay;
          struct Recoveryt{
            int16_t val = 2900; //milliVolts
            uint16_t addr = 0x4484;
          } recovery;
        } CUV;
        struct COVt{ //Cell overvoltage settings
          struct ThresholdLowTempt{
            int16_t val = 4280; //milliVolts
            static const uint16_t addr = 0x4486;
          } thresholdLowTemp;
          struct ThresholdStandardTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x4488;
          } thresholdStandardTemp;
          struct ThresholdHighTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x448A;
          } thresholdHightTemp;
          struct ThresholdRecTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x448C;
          } thresholdRecTemp;
          struct Delayt{
            uint8_t val = 2; //seconds
            uint16_t addr = 0x448E;
          } delay;
          struct RecoveryLowTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x448F;
          } recoveryLowTemp;
          struct RecoveryStandardTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x4491;
          } recoveryStandardTemp;
          struct RecoveryHighTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x4493;
          } recoveryHighTemp;
          struct RecoveryRecTempt{
            int16_t val = 4280; //milliVolts
            uint16_t addr = 0x4495;
          } recoveryRecTemp;
        } COV;
        struct OCC1t{ //Over Current Charge settings
          struct Thresholdt{
            int16_t val = 5000; //milliAmps (positive value is charging, negative is discharging)
            uint16_t addr = 0x4497;
          } threshold;
          struct Delayt{
            uint8_t val = 6; //seconds
            uint16_t addr = 0x4499;
          } delay;
        } OCC1;
        struct OCC2t{
          struct Thresholdt{
            int16_t val = 5900; //milliAmps (positive value is charging, negative is discharging)
            uint16_t addr = 0x449A;
          } threshold ;
          struct Delayt{
            uint8_t val = 3; //seconds
            uint16_t addr = 0x449C;
          } delay;
        } OCC2;
        struct OCCt{
          struct Thresholdt{
            int16_t val = -200; //milliAmps (positive value is charging, negative is discharging)
            uint16_t addr = 0x449D;
          } threshold ;
          struct Delayt{
            uint8_t val = 5; //seconds
            uint16_t addr = 0x449F;
          } delay;
        } OCC;
        struct OCD1t{ //Over Current Discharge settings
          struct Thresholdt{
            int16_t val = -5000; //milliAmps (positive value is charging, negative is discharging)
            static const uint16_t addr = 0x44A0;
          } threshold;
          struct Delayt{
            uint8_t val = 6; //seconds
            static const uint16_t addr = 0x44A2;
          } delay;
        } OCD1;
        struct OCD2t{
          struct Thresholdt{
            int16_t val = -6000; //milliAmps (positive value is charging, negative is discharging)
            uint16_t addr = 0x44A3;
          } threshold ;
          struct Delayt{
            uint8_t val = 3; //seconds
            uint16_t addr = 0x44A5;
          } delay;
        } OCD2;
        struct OCDt{
          struct Thresholdt{
            int16_t val = 200; //milliAmps (positive value is charging, negative is discharging)
            uint16_t addr = 0x44A6;
          } threshold ;
          struct Delayt{
            uint8_t val = 5; //seconds
            uint16_t addr = 0x44A8;
          } delay;
        } OCD;
      } protections;
      struct GasGaugingt{
        struct Designt{
          struct DesignCapacitymAht{
            int16_t val = 2600; //milliAmp hours
            uint16_t addr = 0x444D;
          } designCapacitymAh;
          struct DesignCapacitymWht{
            int16_t val = 3848; //milliWatt hours
            uint16_t addr = 0x444F;
          } designCapacitymWh;
          struct DesignVoltaget{
            int16_t val = 14800; //milliVolts
            uint16_t addr = 0x4451;
          } designVoltage;
        } design;
        struct Statet{
          struct LearnedFCCt{
            int16_t val = 2600;
            uint16_t addr = 0x4100;
          } learnedFCC;
          struct CycleCountt{
            uint16_t val = 3;
            uint16_t addr = 0x4140;
          } cycleCount;
        } state;
        struct CEDVcfgt{
          struct EMFt{
            uint16_t val = 3743;
            uint16_t addr = 0x4590;
          } EMF;
          struct C0t{
            uint16_t val = 149;
            uint16_t addr = 0x4592;
          } C0;
          struct R0t{
            uint16_t val = 867;
            uint16_t addr = 0x4594;
          } R0;
          struct T0t{
            uint16_t val = 4030;
            uint16_t addr = 0x4596;
          } T0;
          struct R1t{
            uint16_t val = 316;
            uint16_t addr = 0x4598;
          } R1;
          struct TCt{
            uint8_t val = 9;
            uint16_t addr = 0x459A;
          } TC;
          struct C1t{
            uint8_t val = 0;
            uint16_t addr = 0x459B;
          } C1;
        } CEDVcfg;
      } gasGauging;
    } ;

 private:
	byte readByteReg( char devAddress, byte regAddress );
	byte readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 );
  boolean readDFBlockReg( char devAddress, byte regAddress1, byte regAddress2, byte *blockData );
  void writeDFByteReg( char devAddress, int16_t regAddress, byte *data, uint8_t arrSize );
	byte readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 );
	uint16_t read2ByteReg( char devAddress, byte regAddress );
  boolean readDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen );
  boolean writeDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen );
	byte readBlockReg( char devAddress, byte regAddress, byte *block );
	void writeByteReg( byte devAddress, byte regAddress, byte dataByte );
	void write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 );
  void readMACblock( char devAddress, byte regAddress, byte *block );
  void writeCommand( char devAddress, byte regAddress );
  void CalculateTable_CRC8();
  byte Compute_CRC8(byte *bytes, uint8_t byteLen);

};
