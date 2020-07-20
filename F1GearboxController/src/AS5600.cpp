//AS5600 Library
//Re-wrote this library due to issues using an ESP32 where values returned were incorrect
//Also added functionality to utilize the 2nd I2C Bus via Wire1

#include "Arduino.h"
#include "AS5600.h"

AS5600::AS5600()
{
	_wire = &Wire;
	_wire->begin();
	_wire->setClock(400000);
}
AS5600::AS5600(TwoWire *the_wire, int sda, int scl)
{
	_wire = the_wire;
	_wire->begin(sda, scl);
	_wire->setClock(400000);
}
bool AS5600::i2cdelay(int size)
{
	int i = 0;
	for (; _wire->available() < size && i <= size; i++)
	{
		delay(2);
	}
	if (i >= size)
	{
		return false;
	}
	else
	{
		return true;
	}
}
int AS5600::getState()
{ //-1:no data, 0:err, 1:ok
	_wire->requestFrom(_AS5600Address, (uint8_t)1);
	if (i2cdelay(1))
	{
		return _wire->read();
	}
	else
	{
		return -1;
	}
}

long AS5600::getVal(byte reg)
{
	_wire->beginTransmission(_AS5600Address); // transmit to device
	_wire->write(reg);						  // sends one byte
	_wire->endTransmission();				  // stop transmitting

	_wire->requestFrom(_AS5600Address, (uint8_t)2);
	int16_t ret = 0;
	if (i2cdelay(2))
	{
		byte *pointer = (byte *)&ret;
		pointer[0] = _wire->read();
		pointer[1] = _wire->read();
	}
	return ret;
}

long AS5600::getVal2(byte registerMSB, byte registerLSB)
{
	_lsb = 0;
	_msb = 0;
	long retVal = -1;
	_wire->beginTransmission(_AS5600Address); // transmit to device
	_wire->write(registerLSB);				  // sends one byte
	_wire->endTransmission();				  // stop transmitting
	delayMicroseconds(10);

	_wire->requestFrom(_AS5600Address, 1);
	if (_wire->available() <= 1)
	{
		_lsb = _wire->read();
	}
	/*
	if (i2cdelay(2))
	{
		_lsb = _wire->read();
	}
*/
	_wire->beginTransmission(_AS5600Address); // transmit to device
	_wire->write(registerMSB);				  // sends one byte
	_wire->endTransmission();				  // stop transmitting

	_wire->requestFrom(_AS5600Address, 1);

	if (_wire->available() <= 1)
	{
		_msb = _wire->read();
	}
	/*
	if (i2cdelay(2))
	{
		_msb = _wire->read();
	}
	*/
	retVal = (_lsb) + (_msb & _msbMask) * 256;
	return retVal;
}

unsigned int AS5600::getPosition()
{

	return getVal2(_RAWANGLEAddressMSB, _RAWANGLEAddressLSB);
}

int AS5600::getAngle()
{
	return getVal2(_ANGLEAddressMSB, _ANGLEAddressLSB);
}
/*
int16_t AS5600::getVal2(byte registerMSB, byte registerLSB)
{

	_wire->beginTransmission(_AS5600Address); // transmit to device
	_wire->write(registerMSB);				  // sends one byte
	_wire->endTransmission();				  // stop transmitting
	delay(10);

	_wire->requestFrom(_AS5600Address, (uint8_t)2);
	int16_t ret = 0;
	if (i2cdelay(2))
	{
		byte *pointer = (byte *)&ret;
		pointer[0] = _wire->read();
		pointer[1] = _wire->read();
	}
	Serial.println(ret);
	return ret;
}
int AS5600::getStatus()
{
	return _getRegister(_STATUSAddress) & 0b00111000;
}

int AS5600::getGain()
{
	return _getRegister(_AGCAddress);
}

int AS5600::getMagnitude()
{
	return _getRegisters2(_MAGNITUDEAddressMSB, _MAGNITUDEAddressLSB);
}
/*
int AS5600::_getRegister(byte register1)
{
	Serial.println("Entered get registers");

	_wire.beginTransmission(_AS5600_AS5600Addressess);
	_wire.write(register1);
	_wire.endTransmission();

	_wire.requestFrom(_AS5600_AS5600Addressess, 1);

	if (_wire.available() <= 1)
	{
		_msb = _wire.read();
	}

	return _msb;
}

long AS5600::_getRegisters2(byte registerMSB, byte registerLSB)
{
	Serial.println("Entered get registers2");
	;

	_lsb = 0;
	_msb = 0;
	long retVal = -1;
	/* Read Low Byte 
	_wire.beginTransmission(_AS5600_AS5600Addressess);
	_wire.write(registerLSB);
	_wire.endTransmission();
	delay(10);

	_wire.requestFrom(_AS5600_AS5600Addressess, 1);
	if (_wire.available() <= 1)
	{
		_lsb = _wire.read();
	}
	/* Read High Byte 
	_wire.beginTransmission(_AS5600_AS5600Addressess);
	_wire.write(registerMSB);
	_wire.endTransmission();
	_wire.requestFrom(_AS5600_AS5600Addressess, 1);

	if (_wire.available() <= 1)
	{
		_msb = _wire.read();
	}
	retVal = (_lsb) + (_msb & _msbMask) * 256;

	Serial.println(retVal);

	return retVal;
}
*/