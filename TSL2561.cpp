#include "TSL2561.h"

#ifdef RasPi
#include <stdlib.h>
#include <math.h>
#endif

/************************************************
 *
 * Compile time settings:
 *
 * Chose the chip type:
 *
 * #define TSL2561_PACKAGE_T_FN_CL
 * *or*
 * #define TSL2561_PACKAGE_CS
 *
 *
 *
 * Calculate LUX using the fast integer aproximation
 * instead of the official formulas. Both calculations
 * are part of the data sheet:
 * #define calculateLux_exampleCodeDatasheet
 *
 ************************************************/

//#define calculateLux_exampleCodeDatasheet

#define TSL2561_PACKAGE_T_FN_CL
//#define TSL2561_PACKAGE_CS


// used to control data transfer from/to TSL2561
#define TSL2561_COMMAND_BIT       (0x80)	// Must be 1
#define TSL2561_CLEAR_BIT         (0x40)	// Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)	// 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)	// 1 = using block read/write

// register 0: Control register, only 2 bits
#define TSL2561_CONTROL_POWERON   (0x03)
#define TSL2561_CONTROL_POWEROFF  (0x00)

// constants from datasheet to calculate lux values
#ifdef TSL2561_PACKAGE_CS
#define TSL2561_INT_K1C           (0x0043)	// 0.130 * 2^RATIO_SCALE
#define TSL2561_INT_B1C           (0x0204)	// 0.0315 * 2^LUXSCALE
#define TSL2561_INT_M1C           (0x01ad)	// 0.0262 * 2^LUXSCALE
#define TSL2561_INT_K2C           (0x0085)	// 0.260 * 2^RATIO_SCALE
#define TSL2561_INT_B2C           (0x0228)	// 0.0337 * 2^LUXSCALE
#define TSL2561_INT_M2C           (0x02c1)	// 0.0430 * 2^LUXSCALE
#define TSL2561_INT_K3C           (0x00c8)	// 0.390 * 2^RATIO_SCALE
#define TSL2561_INT_B3C           (0x0253)	// 0.0363 * 2^LUXSCALE
#define TSL2561_INT_M3C           (0x0363)	// 0.0529 * 2^LUXSCALE
#define TSL2561_INT_K4C           (0x010a)	// 0.520 * 2^RATIO_SCALE
#define TSL2561_INT_B4C           (0x0282)	// 0.0392 * 2^LUXSCALE
#define TSL2561_INT_M4C           (0x03df)	// 0.0605 * 2^LUXSCALE
#define TSL2561_INT_K5C           (0x014d)	// 0.65 * 2^RATIO_SCALE
#define TSL2561_INT_B5C           (0x0177)	// 0.0229 * 2^LUXSCALE
#define TSL2561_INT_M5C           (0x01dd)	// 0.0291 * 2^LUXSCALE
#define TSL2561_INT_K6C           (0x019a)	// 0.80 * 2^RATIO_SCALE
#define TSL2561_INT_B6C           (0x0101)	// 0.0157 * 2^LUXSCALE
#define TSL2561_INT_M6C           (0x0127)	// 0.0180 * 2^LUXSCALE
#define TSL2561_INT_K7C           (0x029a)	// 1.3 * 2^RATIO_SCALE
#define TSL2561_INT_B7C           (0x0037)	// 0.00338 * 2^LUXSCALE
#define TSL2561_INT_M7C           (0x002b)	// 0.00260 * 2^LUXSCALE
#define TSL2561_INT_K8C           (0x029a)	// 1.3 * 2^RATIO_SCALE
#define TSL2561_INT_B8C           (0x0000)	// 0.000 * 2^LUXSCALE
#define TSL2561_INT_M8C           (0x0000)	// 0.000 * 2^LUXSCALE

static const uint16_t TSL2561_INT_K[8] = {
	TSL2561_INT_K1C,
	TSL2561_INT_K2C,
	TSL2561_INT_K3C,
	TSL2561_INT_K4C,
	TSL2561_INT_K5C,
	TSL2561_INT_K6C,
	TSL2561_INT_K7C,
	TSL2561_INT_K8C,
};
static const uint16_t TSL2561_INT_M[8] = {
	TSL2561_INT_M1C,
	TSL2561_INT_M2C,
	TSL2561_INT_M3C,
	TSL2561_INT_M4C,
	TSL2561_INT_M5C,
	TSL2561_INT_M6C,
	TSL2561_INT_M7C,
	TSL2561_INT_M8C,
};
static const uint16_t TSL2561_INT_B[8] = {
	TSL2561_INT_B1C,
	TSL2561_INT_B2C,
	TSL2561_INT_B3C,
	TSL2561_INT_B4C,
	TSL2561_INT_B5C,
	TSL2561_INT_B6C,
	TSL2561_INT_B7C,
	TSL2561_INT_B8C,
};
#else //!TSL2561_PACKAGE_CS
// T, FN and CL package values
#define TSL2561_INT_K1T           (0x0040)	// 0.125 * 2^RATIO_SCALE
#define TSL2561_INT_B1T           (0x01f2)	// 0.0304 * 2^LUXSCALE
#define TSL2561_INT_M1T           (0x01be)	// 0.0272 * 2^LUXSCALE
#define TSL2561_INT_K2T           (0x0080)	// 0.250 * 2^RATIO_SCALE
#define TSL2561_INT_B2T           (0x0214)	// 0.0325 * 2^LUXSCALE
#define TSL2561_INT_M2T           (0x02d1)	// 0.0440 * 2^LUXSCALE
#define TSL2561_INT_K3T           (0x00c0)	// 0.375 * 2^RATIO_SCALE
#define TSL2561_INT_B3T           (0x023f)	// 0.0351 * 2^LUXSCALE
#define TSL2561_INT_M3T           (0x037b)	// 0.0544 * 2^LUXSCALE
#define TSL2561_INT_K4T           (0x0100)	// 0.50 * 2^RATIO_SCALE
#define TSL2561_INT_B4T           (0x0270)	// 0.0381 * 2^LUXSCALE
#define TSL2561_INT_M4T           (0x03fe)	// 0.0624 * 2^LUXSCALE
#define TSL2561_INT_K5T           (0x0138)	// 0.61 * 2^RATIO_SCALE
#define TSL2561_INT_B5T           (0x016f)	// 0.0224 * 2^LUXSCALE
#define TSL2561_INT_M5T           (0x01fc)	// 0.0310 * 2^LUXSCALE
#define TSL2561_INT_K6T           (0x019a)	// 0.80 * 2^RATIO_SCALE
#define TSL2561_INT_B6T           (0x00d2)	// 0.0128 * 2^LUXSCALE
#define TSL2561_INT_M6T           (0x00fb)	// 0.0153 * 2^LUXSCALE
#define TSL2561_INT_K7T           (0x029a)	// 1.3 * 2^RATIO_SCALE
#define TSL2561_INT_B7T           (0x0018)	// 0.00146 * 2^LUXSCALE
#define TSL2561_INT_M7T           (0x0012)	// 0.00112 * 2^LUXSCALE
#define TSL2561_INT_K8T           (0x029a)	// 1.3 * 2^RATIO_SCALE
#define TSL2561_INT_B8T           (0x0000)	// 0.000 * 2^LUXSCALE
#define TSL2561_INT_M8T           (0x0000)	// 0.000 * 2^LUXSCALE
static const uint16_t TSL2561_INT_K[8] = {
	TSL2561_INT_K1T,
	TSL2561_INT_K2T,
	TSL2561_INT_K3T,
	TSL2561_INT_K4T,
	TSL2561_INT_K5T,
	TSL2561_INT_K6T,
	TSL2561_INT_K7T,
	TSL2561_INT_K8T,
};
static const uint16_t TSL2561_INT_M[8] = {
	TSL2561_INT_M1T,
	TSL2561_INT_M2T,
	TSL2561_INT_M3T,
	TSL2561_INT_M4T,
	TSL2561_INT_M5T,
	TSL2561_INT_M6T,
	TSL2561_INT_M7T,
	TSL2561_INT_M8T,
};
static const uint16_t TSL2561_INT_B[8] = {
	TSL2561_INT_B1T,
	TSL2561_INT_B2T,
	TSL2561_INT_B3T,
	TSL2561_INT_B4T,
	TSL2561_INT_B5T,
	TSL2561_INT_B6T,
	TSL2561_INT_B7T,
	TSL2561_INT_B8T,
};
#endif //!TSL2561_PACKAGE_CS

// Constants from datasheet for integer calculations

// In order to avoid overflows, CHSCALE is not used
// in integer calculations, but computed out while
// computing the scales value.

#define TSL2561_INT_RATIOSCALE    (9)      // Scale ratio by 2^9
#define TSL2561_INT_CHSCALE       (10)     // Scale channel values by 2^10
#define TSL2561_INT_CHSCALE_TINT0 (0x7517) // 322/11 * 2^TSL2561_INT_CHSCALE
#define TSL2561_INT_CHSCALE_TINT1 (0x0FE7) // 322/81 * 2^TSL2561_INT_CHSCALE

// TSL2561_INT_LUXSCALE is for integer calculations only:
#ifdef calculateLux_exampleCodeDatasheet
#define TSL2561_INT_LUXSCALE      (14)     // Scale by 2^14
#else
#define TSL2561_INT_LUXSCALE      (0)
#endif

// These values are calculate for for gain==16; multiply by 16 if gain == 1
static const float scales[3] = {
	1.0*TSL2561_INT_CHSCALE_TINT0
		/(1UL << (TSL2561_INT_LUXSCALE + TSL2561_INT_CHSCALE)),
	1.0*TSL2561_INT_CHSCALE_TINT1
		/(1UL << (TSL2561_INT_LUXSCALE + TSL2561_INT_CHSCALE)),
	1.0*(1UL << TSL2561_INT_CHSCALE)
		/(1UL << (TSL2561_INT_LUXSCALE + TSL2561_INT_CHSCALE)),
};

const word TSL2561_DELAY_INTTIME[3] = {
	TSL2561_DELAY_INTTIME_13MS,
	TSL2561_DELAY_INTTIME_101MS,
	TSL2561_DELAY_INTTIME_402MS,
};

static const word TSL2561_GAINUPGRADE[3] = {
	205,
	1542,
	2730
};

const word TSL2561_CLIP[3] = {
	 4900,
	37000,
	65535
};

enum {
	TSL2561_REGISTER_CONTROL = 0x00,
	TSL2561_REGISTER_TIMING = 0x01,
	TSL2561_REGISTER_THRESHHOLDL_LOW = 0x02,
	TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03,
	TSL2561_REGISTER_THRESHHOLDH_LOW = 0x04,
	TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05,
	TSL2561_REGISTER_INTERRUPT = 0x06,
	TSL2561_REGISTER_CRC = 0x08,
	TSL2561_REGISTER_ID = 0x0A,
	TSL2561_REGISTER_CHAN0_LOW = 0x0C,
	TSL2561_REGISTER_CHAN0_HIGH = 0x0D,
	TSL2561_REGISTER_CHAN1_LOW = 0x0E,
	TSL2561_REGISTER_CHAN1_HIGH = 0x0F
};

void TSL2561::writeI2C8(uint8_t reg, byte value)
{
	Wire.beginTransmission(_addr);
	wireWrite(reg);
	wireWrite(value);
	Wire.endTransmission();
}

/*uint8_t TSL2561::readI2C8(uint8_t reg)
{
	Wire.beginTransmission(_addr);
	wireWrite(reg);
	Wire.endTransmission();
	Wire.requestFrom(_addr, 1);
	return wireRead();
}*/
/*uint16_t TSL2561::readI2C16(uint8_t reg)
{
	uint16_t x;
	uint16_t t;

	Wire.beginTransmission(_addr);
	wireWrite(reg);
	Wire.endTransmission();

	Wire.requestFrom(_addr, 2);
	t = wireRead();
	x = wireRead();
	x <<= 8;
	x |= t;
	return x;
}*/

void TSL2561::enableDevice(void)
{
	if (is_powered)
		return;
	writeI2C8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
		   TSL2561_CONTROL_POWERON);
	is_powered = true;
}
void TSL2561::disableDevice(void)
{
	writeI2C8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
		   TSL2561_CONTROL_POWEROFF);
	is_powered = false;
}

// read sensor data
bool TSL2561::getDataRaw()
{
	Wire.beginTransmission(_addr);
	// read block starting at register 0xc:
	Wire.write(TSL2561_COMMAND_BIT
	         | TSL2561_BLOCK_BIT 
	         | TSL2561_REGISTER_CHAN0_LOW);
	Wire.endTransmission();
	if (Wire.requestFrom(_addr, (byte)4) != 4)
		return false;
	sensorData.broadband = Wire.read() | Wire.read() << 8;
	sensorData.ir        = Wire.read() | Wire.read() << 8;
	luxIsCurrent = false;
	millis_lastread = millis();
	return true;
}

bool TSL2561::getDataDelay()
{
	bool err;
	enableDevice();
	delay(TSL2561_DELAY_INTTIME[_integrationTime]);
	err = getDataRaw();
	if (!keep_powered)
		disableDevice();
  Serial.print(sensorData.broadband);
  Serial.print(" ");
  Serial.print(sensorData.ir);
  Serial.println(" read bb ir");
	return err;
}
void TSL2561::setGainIntegrationTime(
 byte gain,
 byte time)
{
  _gain = gain;
  _integrationTime = time;
  _scale = scales[_integrationTime];
  if (_gain == TSL2561_GAIN_1X)
    _scale *= 16.0;
  writeI2C8(TSL2561_COMMAND_BIT
          | TSL2561_REGISTER_TIMING,
            time | gain);
  Serial.print(time);
  Serial.print(" ");
  Serial.print(gain);
  Serial.println(" Set t g");
}

// maxtime == 0xfe: only increase to match current value
// maxtime == 0xff: copy time parameter
// time must be one of TSL2561_INTEGRATIONTIME_{13,101,402}MS
void TSL2561::setGainIntegrationTimes(
 byte gain,
 byte time,
 byte maxtime)
{
  if (maxtime == 0xfe) {
    if (_integrationMaxTime < time)
      _integrationMaxTime = time;
  } else  if (maxtime == 0xff) {
      _integrationMaxTime = time;
  } else {
    _integrationMaxTime = maxtime;
  }
  return setGainIntegrationTime(gain, time);
}

// false: read error
// true: read OK, but maybe did overflow
bool TSL2561::updateDataAutorange()
{
	// If we restart, we'll use the fastest settings.
	// We will not restart when using the fastest setting,
	// so there is no loop
	restart_with_min_setting:
	bool readOK = updateDataDelay(); // optimistically use old setting
	if (!readOK)
		return readOK;

	uint16_t maxval = max(sensorData.broadband, sensorData.ir);
	if (maxval >= TSL2561_CLIP[_integrationTime]) {
		if (_integrationTime == TSL2561_INTEGRATIONTIME_13MS
		 && _gain == TSL2561_GAIN_1X)
			// we can't shorten the exposure, thus we don't
			return true;
		setGainIntegrationTime(TSL2561_GAIN_1X, TSL2561_INTEGRATIONTIME_13MS);
		goto restart_with_min_setting;
	}
	// now: maxval < saturation, therefore
	// this is a good reading, we safe it
	struct SensorDataRaw sd_good = sensorData;
	byte downgraded_from_t = _integrationTime;
	byte downgraded_from_g = _gain;

	// most likely new values, we will update them when needed
	byte g = TSL2561_GAIN_16X;
	byte t = _integrationMaxTime;

	if (_integrationTime == _integrationMaxTime) {
		if (_gain == TSL2561_GAIN_16X)
			return true; // no more gain possible
		if (maxval > TSL2561_GAINUPGRADE[_integrationTime])
			return true; // value OK
		goto do_upgrade; // just set the gain, we are at max
	}

	//  _integrationTime < _integrationMaxTime
	// therefore _integrationTime != TSL2561_INTEGRATIONTIME_402MS

	if (_integrationTime == TSL2561_INTEGRATIONTIME_101MS)
		 maxval = (long)maxval * 11/81;
	if (_gain == TSL2561_GAIN_16X)
		maxval /= 16;

	// "carefully" calculated using a spreadsheet
	// and 2/3 max as the target value
	if (maxval <= 93 && _integrationMaxTime == TSL2561_INTEGRATIONTIME_402MS) {
		/* g=TSL2561_GAIN_16X; t=TSL2561_INTEGRATIONTIME_402MS;*/
	} else if (maxval <= 209) {
		/*g=TSL2561_GAIN_16X*/; t=TSL2561_INTEGRATIONTIME_101MS;
	} else if (maxval <= 1492 && _integrationMaxTime == TSL2561_INTEGRATIONTIME_402MS) {
		g=TSL2561_GAIN_1X; /*t=TSL2561_INTEGRATIONTIME_402MS;*/
	} else
		// the current setting is fine
		return true;

	do_upgrade:
	setGainIntegrationTime(g, t);
	did_upgrade:
	readOK = getDataDelay();
	if (!readOK)
		return readOK;
	maxval = max(sensorData.broadband, sensorData.ir);
	if (maxval >= TSL2561_CLIP[_integrationTime]) {
		sensorData = sd_good; // use the good reading.
		// set timing values and scale (important)
		setGainIntegrationTime(downgraded_from_g, downgraded_from_t);
	}
	return true;
}

float TSL2561::calculateLux()
{
	unsigned long chScale;
	word const broadband = sensorData.broadband;
	word const ir = sensorData.ir;
	
	if (luxIsCurrent)
		return lux;

	if (max(broadband, ir) >= TSL2561_CLIP[_integrationTime])
		return -1; // overflow

#ifdef calculateLux_exampleCodeDatasheet
	// faster and smaller calculation

	// Find the ratio of the channel values (Channel1/Channel0)
	word ratio = 0;
	if (broadband != 0){
		// calculate ratio with rounding: (2a/b + 1)/2
		ratio = (((long)ir << (TSL2561_INT_RATIOSCALE + 1)) / broadband + 1) / 2;
		// handle result overflow; shouldn't happen normally
		if (ir/2 > broadband)
			ratio = 2048; // higher than any limit we use
	}

	// find index
	byte i;
	for (i = 0; i < 7; i++) // find table entry
		if (ratio <= TSL2561_INT_K[i])
			break;
	float luxUnscaled = (long)broadband * TSL2561_INT_B[i] 
	                  - (long)ir        * TSL2561_INT_M[i];

	// Do not allow negative lux value
	if (luxUnscaled < 0)
		luxUnscaled = 0;
#else //!calculateLux_smallFast
	// calculate Luy using official formula
	float luxUnscaled;
	float ratio2 = ir * 1.0 / broadband;
/*  Serial.print(broadband);Serial.print(" ");
  Serial.print(ir);Serial.print(" ");
  Serial.print(_integrationTime);Serial.print(" ");
  Serial.print(_gain);Serial.print(" ");
  Serial.print(ratio2);Serial.print(" ");
  Serial.print(_scale*1000);Serial.println(" bb ir t g rat sc*1000");*/
	if (ratio2 < 0.5) {
		luxUnscaled = 0.0304 * broadband - 0.062 * broadband * pow(ratio2, 1.4);
	} else if (ratio2 < 0.61) {
		luxUnscaled = 0.0224 * broadband - 0.031 * ir;
	} else if (ratio2 < 0.8) {
		luxUnscaled = 0.0128 * broadband - 0.0153 * ir;
	} else if (ratio2 < 1.3) {
		luxUnscaled = 0.00146 * broadband - 0.00112 * ir;
	} else
		luxUnscaled = 0;
	#endif
	luxIsCurrent = true;
	return lux = luxUnscaled * _scale;
}

