/**************************************************************************/
/*!
    @file     Lorro_BQ4050.h
    @author   Lorro


	A library for interfacing with TI BQ4050 battery fuel gauge chip


*/
/**************************************************************************/

 #include "Arduino.h"


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
#define operationStatusReg                    0x54
#define FETcontrol                            0x22
#define deviceResetReg                        0x41

#define DFProtectionsCUVThreshold

class Lorro_BQ4050{
 public:
	Lorro_BQ4050( char addr );
  void init();
 	// Lorro_BQ4050(  );
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
  boolean getOperationStatus(byte *opStatArr);
  boolean getXCHGstatus();
  boolean getXDSGstatus();
  boolean getPCHGstatus();
  boolean getCHGstatus();
  boolean getDSGstatus();
  void FETtoggle();
  void deviceReset();
  template<typename T>
  void writeFlash(const T& dataParam);
  void writeThreshold();
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

struct DFt{
  struct Protectionst{
    struct CUVt{ //Cell undervoltage settings
      struct Thresholdt{
        int16_t val = 2850; //milliVolts
        uint16_t addr = 0x4481;
      } Threshold;
      struct Delayt{
        uint8_t val = 2; //seconds
        uint16_t addr = 0x4483;
      } Delay;
      struct Recoveryt{
        int16_t val = 2900; //milliVolts
        uint16_t addr = 0x4484;
      } Recovery;
    } CUV;
    struct COVt{ //Cell overvoltage settings
      struct ThresholdLowTempt{
        int16_t val = 4280; //milliVolts
        static const uint16_t addr = 0x4486;
      } ThresholdLowTemp;
      struct ThresholdStandardTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x4488;
      } ThresholdStandardTemp;
      struct ThresholdHighTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x448A;
      } ThresholdHightTemp;
      struct ThresholdRecTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x448C;
      } ThresholdRecTemp;
      struct Delayt{
        uint8_t val = 2; //seconds
        uint16_t addr = 0x448E;
      } Delay;
      struct RecoveryLowTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x448F;
      } RecoveryLowTemp;
      struct RecoveryStandardTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x4491;
      } RecoveryStandardTemp;
      struct RecoveryHighTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x4493;
      } RecoveryHighTemp;
      struct RecoveryRecTempt{
        int16_t val = 4280; //milliVolts
        uint16_t addr = 0x4495;
      } RecoveryRecTemp;
    } COV;
    struct OCC1t{ //Over Current Charge settings
      struct Thresholdt{
        int16_t val = 5000; //milliAmps (positive value is charging, negative is discharging)
        uint16_t addr = 0x4497;
      } Threshold;
      struct Delayt{
        uint8_t val = 6; //seconds
        uint16_t addr = 0x4499;
      } Delay;
    } OCC1;
    struct OCC2t{
      struct Thresholdt{
        int16_t val = 6000; //milliAmps (positive value is charging, negative is discharging)
        uint16_t addr = 0x449A;
      } Threshold ;
      struct Delayt{
        uint8_t val = 3; //seconds
        uint16_t addr = 0x449C;
      } Delay;
    } OCC2;
    struct OCC{
      struct Thresholdt{
        int16_t val = -200; //milliAmps (positive value is charging, negative is discharging)
        uint16_t addr = 0x449D;
      } Threshold ;
      struct Delayt{
        uint8_t val = 5; //seconds
        uint16_t addr = 0x449F;
      } Delay;
    } OCC;
    struct OCD1t{ //Over Current Discharge settings
      struct Thresholdt{
        int16_t val = -5100; //milliAmps (positive value is charging, negative is discharging)
        static const uint16_t addr = 0x44A0;
      } Threshold;
      struct Delayt{
        uint8_t val = 6; //seconds
        static const uint16_t addr = 0x44A2;
      } Delay;
    } OCD1;
    struct OCD2t{
      struct Thresholdt{
        int16_t val = -6000; //milliAmps (positive value is charging, negative is discharging)
        uint16_t addr = 0x44A3;
      } Threshold ;
      struct Delayt{
        uint8_t val = 3; //seconds
        uint16_t addr = 0x44A5;
      } Delay;
    } OCD2;
    struct OCDt{
      struct Thresholdt{
        int16_t val = 200; //milliAmps (positive value is charging, negative is discharging)
        uint16_t addr = 0x44A6;
      } Threshold ;
      struct Delayt{
        uint8_t val = 5; //seconds
        uint16_t addr = 0x44A8;
      } Delay;
    } OCD;
  } Protections;
  struct GasGaugingt{
    struct Designt{
      struct DesignCapacitymAht{
        int16_t val = 2600; //milliAmp hours
        uint16_t addr = 0x444D;
      } DesignCapacitymAh;
      struct DesignCapacitymWht{
        int16_t val = 3848; //milliWatt hours
        uint16_t addr = 0x444F;
      } DesignCapacitymWh;
      struct DesignVoltaget{
        int16_t val = 14800; //milliVolts
        uint16_t addr = 0x4451;
      } DesignVoltage;
    } Design;
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
  } GasGauging;
} DF;


 private:
	byte readByteReg( char devAddress, byte regAddress );
	byte readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 );
  boolean readDFBlockReg( char devAddress, byte regAddress1, byte regAddress2, byte *blockData );
  void writeDFByteReg( char devAddress, int16_t regAddress, byte *data, uint8_t arrSize );
	byte readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 );
	uint16_t read2ByteReg( char devAddress, byte regAddress );
	byte readBlockReg( char devAddress, byte regAddress, byte *block );
	void writeByteReg( byte devAddress, byte regAddress, byte dataByte );
	void write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 );
  void readMACblock( char devAddress, byte regAddress, byte *block );
  void writeCommand( char devAddress, byte regAddress );
  void CalculateTable_CRC8();
  byte Compute_CRC8(byte *bytes, uint8_t byteLen);

};
