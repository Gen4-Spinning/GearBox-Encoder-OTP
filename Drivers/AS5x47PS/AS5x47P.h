#include "main.h"
#include <stdint.h>
#include <stdbool.h>
//start private variables

//Registers declaration 

//Volatile Registers Addresses
#define NOP_REG		0x0000
#define ERRFL_REG 	0x0001
#define PROG_REG	0x0003
#define DIAGAGC_REG 	0x3FFC
#define MAG_REG 	0x3FFD
#define ANGLE_REG 	0x3FFE
#define ANGLECOM_REG 	0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 	0x0016
#define ZPOSL_REG 	0x0017
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019

#define WRITE		0
#define READ		1

#define NOP_FRAME 0xC000
#define ANGLE_READ_FRAME 0x7FFE
// ERRFL Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t frerr:1;
    uint16_t invcomm:1;
    uint16_t parerr:1;
    uint16_t unused:13;
  } values;
} Errfl;


// PROG Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t progen:1;
    uint16_t unused:1;
    uint16_t otpref:1;
    uint16_t progotp:1;
    uint16_t unused1:2;
    uint16_t progver:1;
    uint16_t unused2:9;
  } values;
} Prog;

// DIAAGC Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t agc:8;
    uint16_t lf:1;
    uint16_t cof:1;
    uint16_t magh:1;
    uint16_t magl:1;
    uint16_t unused:4;
  } values;
} Diaagc;

// MAG Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t cmag:14;
    uint16_t unused:2;
  } values;
} Mag;

// ANGLE Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t cordicang:14;
    uint16_t unused:2;
  } values;
} Angle;

// ANGLECOM Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t daecang:14;
    uint16_t unused:2;
  } values;
} Anglecom;


// ZPOSM Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint16_t zposm:8;
    uint16_t unused:6;
  } values;
} Zposm;

// ZPOSL Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint16_t zposl:6;
    uint16_t compLerrorEn:1;
    uint16_t compHerrorEn:1;
    uint16_t unused:6;
  } values;
} Zposl;

// SETTINGS1 Register Definition
typedef union {
	uint16_t raw;
  struct __attribute__ ((packed)) {
	  uint16_t factorySetting:1;
	  uint16_t not_used:1;
	  uint16_t dir:1;
	  uint16_t uvw_abi:1;
	  uint16_t daecdis:1;
	  uint16_t abibin:1;
	  uint16_t dataselect:1;
	  uint16_t pwmon:1;
  } values;
} Settings1;

// SETTINGS2 Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t uvwpp:3;
    uint8_t hys:2;
    uint8_t abires:3;
  } values;
} Settings2;


// Command Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t commandFrame:14;
    uint16_t rw:1;
    uint16_t parc:1;
  } values;
} CommandFrame;

// ReadData Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t data:14;
    uint16_t ef:1;
    uint16_t pard:1;
  } values;
} ReadDataFrame;

// WriteData Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__ ((packed)) {
    uint16_t data:14;
    uint16_t low:1;
    uint16_t pard:1;
  } values;
} WriteDataFrame;

typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t data:6;
    uint8_t comp_l_error:1;
    uint8_t comp_h_error:1;
  } values;
} ZPOSL_frame;

typedef union {
  uint8_t raw;
  struct __attribute__ ((packed)) {
    uint8_t data:8;
  } values;
} ZPOSH_frame;

//end private varibles
void configureERRFL(void);
void settings1Init(void);
void writeSettings1(Settings1 values);
void writeSettings2(Settings2 values);
uint16_t readData(uint16_t command, uint16_t nopCommand);
void writeData(uint16_t address, uint16_t value);
bool parityCheck(uint16_t data);
ReadDataFrame readRegister(uint16_t registerAddress);
void writeRegister(uint16_t registerAddress, uint16_t registerValue);
void writeToRegister(uint16_t address, uint16_t value);
void readFromRegister(uint16_t address);
void writeZeroPosition(Zposm zposm, Zposl zposl);
uint16_t angleReading();


uint16_t AS5047_SPI_Read(uint16_t command, uint8_t continuousRead);
uint8_t AS5047_SPI_Write(uint16_t address, uint16_t value);
uint16_t AS5047_readRegister(uint16_t registerAddress,uint8_t continuousRead);
void AS5047_writeRegister(uint16_t registerAddress, uint16_t registerValue);
uint8_t checkReadForError(uint16_t data);

void writeZeroReg(uint16_t zeroValue);
uint16_t getProgrammedZeroOffset(void);
void SetupABIwithoutPWM(void);
uint8_t Check_ABI_SetCorrectly(Settings1 settings1, Settings2 settings2);

//uint16_t GetAveragedAngleReading(void);
uint16_t GetAveragedAngleReading(uint16_t rdngs);
