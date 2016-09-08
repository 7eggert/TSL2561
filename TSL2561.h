#ifndef _included_TSL2561_H_
#define _included_TSL2561_H_

#ifdef RasPi

#include "piArduino.h"
#include "piWire.h"

#else	//!RasPi

#if ARDUINO >= 100
#include <Arduino.h>
#else // old ARDUINO
#include <WProgram.h>
#endif // old ARDUINO

#include <AutoWire.h>

#endif //!RasPi

// I2C address
#define TSL2561_ADDR_LOW          (0x29)
#define TSL2561_ADDR_FLOAT        (0x39)	// Default address (pin left floating)
#define TSL2561_ADDR_HIGH         (0x49)

// delay times for reading the sensordata after enabling the device
// array index coincides with TSL2561_INTEGRATIONTIME_...MS
#define TSL2561_DELAY_INTTIME_13MS    (15)
#define TSL2561_DELAY_INTTIME_101MS   (120)
#define TSL2561_DELAY_INTTIME_402MS   (450)
extern const word TSL2561_DELAY_INTTIME[3];
extern const word TSL2561_CLIP[3];

enum {
	TSL2561_INTEGRATIONTIME_13MS = 0x00,  // 13.7ms
	TSL2561_INTEGRATIONTIME_101MS = 0x01, // 101ms
	TSL2561_INTEGRATIONTIME_402MS = 0x02, // 402ms
	 // Do not use the following values for the time parameter.
	 // They are handled specially in the setter function.
	 // maxtime: copy from time parameter:
	TSL2561_INTEGRATIONTIME_COPY = 0xff,
	 // maxtime: set to max(time, _integrationTime):
	TSL2561_INTEGRATIONTIME_MAX = 0xfe,
};


enum {
	TSL2561_GAIN_1X  = 0x00, // No gain
	TSL2561_GAIN_16X = 0x10, // 16x gain
};


class TSL2561 {
	public:
		struct SensorDataRaw {
			uint16_t broadband;
			uint16_t ir;
		} sensorData;
		float lux; // only updates if you call calculateLux();
		bool luxIsCurrent; // calculateLux() was called
		unsigned long millis_lastread;
		bool is_powered;
		bool keep_powered; // does not affect disableDevice()

	TSL2561()
	: luxIsCurrent(false),
	  keep_powered(false) // power-save between readings
	{ }; // only set fixed values, everything else is set on begin();

	// Requires Wire.begin() to be called
	// Does not work within a constructor for global static variables
	// because system is not initialized by then
	void begin(
		byte addr,
		byte gain    = TSL2561_GAIN_1X,
		byte time    = TSL2561_INTEGRATIONTIME_13MS,
		byte maxtime = TSL2561_INTEGRATIONTIME_COPY)
	{
		_addr = addr;
		millis_lastread = millis() - 1000; // outdated
		setGainIntegrationTimes(gain, time, maxtime);
		disableDevice();
	};

	// functions to set gain and integration time;
	// don't use update...() functions directly after
	// these functions, call getData...() instead

	// only sets the current integration time and gain
	// If you don't use getDataAutorange() or updateDataAutorange(),
	// use this
	void setGainIntegrationTime(byte gain, byte time);
	// updates gain; update intgration time and it's max value fo autoRange
	void setGainIntegrationTimes(byte gain, byte time, byte maxtime = TSL2561_INTEGRATIONTIME_COPY);
	void setIntegrationTimes(byte time, byte maxtime = TSL2561_INTEGRATIONTIME_COPY) {
		return setGainIntegrationTimes(_gain, time, maxtime);
	};
	void setGain(byte gain) {
		return setGainIntegrationTime(gain, _integrationTime);
	};

	// turn sensor on and off
	void enableDevice(void);
	void disableDevice(void);

	// the following functions will return false if a
	// communication error occurs, true otherwise
	//
	// updateData... will use an available value,
	// getData... will always query the device.
	//
	// Don't just enableDevice() then call update...;
	// instead use keep_powered to prevent getDataDelay() from
	// disabling it, or use e.g. delay(getIntegrationDelay())
	// to ensure sufficient time for valid readings.

	// This is the most raw function, just read the data from the
	// registers. The data is possily invalid.
	bool getDataRaw();    

	// Enable device, wait, read, possibly disable it
	bool getDataDelay();
	bool getDataAutorange() { // get best reading allowed
		return getDataDelay()
		    && updateDataAutorange();
	};

	bool updateDataAutorange();
	// worst case: 2 * 450 + 15 ms for maxtime == TSL2561_INTEGRATIONTIME_402MS
	// worst case: 2 * 120 + 15 ms for maxtime == TSL2561_INTEGRATIONTIME_101MS
	// worst case: 2 * 15 ms       for maxtime == TSL2561_INTEGRATIONTIME_13MS
	// (only counting delay() )
	// New device settings will stay
	bool updateDataRaw() {
		if (millis() - millis_lastread < TSL2561_DELAY_INTTIME[_integrationMaxTime])
			return true; // no read error
		return getDataRaw();
	};
	bool updateDataDelay() {
		if (millis() - millis_lastread < TSL2561_DELAY_INTTIME[_integrationMaxTime])
			return true; // no read error
		if (is_powered)
			return getDataRaw();
		return getDataDelay();
	};
	
	byte getGainRaw() { return _gain; }
	byte getIntegrationTimeRaw() { return _integrationTime; }
	byte getIntegrationTimeMaxRaw() { return _integrationMaxTime; }

	// numeric values
	byte getGain() { return _gain? _gain : 1; }
	byte getIntegrationDelay()
		{ return TSL2561_DELAY_INTTIME[_integrationTime]; }
	byte getIntegrationDelayMax()
		{ return TSL2561_DELAY_INTTIME[_integrationMaxTime]; }

	// current clipping value of sensor
	word getSensorClip() { return TSL2561_CLIP[_integrationTime]; }

	float calculateLux(); // calculate lux from existing data
	
	private:
	byte _addr;
	byte _integrationTime;
	byte _integrationMaxTime;
	byte _gain;
		float _scale;

	void writeI2C8(uint8_t reg, byte value);
	//uint8_t readI2C8(uint8_t reg);
	//uint16_t readI2C16(uint8_t reg);
};
#endif
