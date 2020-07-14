#include "Arduino.h"
#include "AS5600.h"

AS5600::AS5600()
{
  //set up AS5600
  myWire.begin();
}
AS5600::AS5600(int sclPin, int sdaPin)
{
 myWire = Wire1;
 myWire.begin(sdaPin, sclPin);
}
long AS5600::getPosition()
{
  return _getRegisters2(_RAWANGLEAddressMSB, _RAWANGLEAddressLSB);
}

int AS5600::getAngle()
{
  return _getRegisters2(_ANGLEAddressMSB, _ANGLEAddressLSB);
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

int AS5600::_getRegister(byte register1)
{
  myWire.beginTransmission(_AS5600Address);
  myWire.write(register1);
  myWire.endTransmission();

  myWire.requestFrom(_AS5600Address, 1);

  if (myWire.available() <= 1)
  {
    _msb = myWire.read();
  }

  return _msb;
}

long AS5600::_getRegisters2(byte registerMSB, byte registerLSB)
{
  _lsb = 0;
  _msb = 0;

  myWire.beginTransmission(_AS5600Address);
  myWire.write(registerMSB);
  myWire.endTransmission();
  delay(10);

  myWire.requestFrom(_AS5600Address, 1);

  if (myWire.available() <= 1)
  {
    _msb = myWire.read();
  }

  myWire.requestFrom(_AS5600Address, 1);

  myWire.beginTransmission(_AS5600Address);
  myWire.write(registerLSB);
  myWire.endTransmission();

  if (myWire.available() <= 1)
  {
    _lsb = myWire.read();
  }

  return (_lsb) + (_msb & _msbMask) * 256;
}
