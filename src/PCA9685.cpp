
#include "PCA9685.h"
#include <wiringPiI2C.h>
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

using namespace std;

PCA9685::PCA9685(int device, int freq)
{
	if ( (fd = wiringPiI2CSetup(device)) < 0)
	{
		cout<<"error opening i2c channel"<<endl;
	}
	
	reset();
	setPWMFreq(freq);
}

void PCA9685::reset()
{
	writeByte(MODE1, 0x00);
	writeByte(MODE2, 0x04);
}

void PCA9685::writeByte(uint8_t reg, uint8_t value)
{
	int err;
	if( (err = wiringPiI2CWriteReg8(fd, reg, value)) < 0)
	cout<<"Unable to write"<<endl;
	
}

void PCA9685::writeWord(uint8_t reg, uint16_t value)
{
	writeByte(reg, value & 0xff);
	writeByte(reg+1, value >> 8);
}

void PCA9685::setPWMFreq(int frequency)
{
	uint8_t prescale = (CLOCK/4096/frequency) - 1;
	//cout<<"prescale: "<<(int)prescale<<endl;
	writeByte(MODE1, 0x10);		//sleep
	usleep(1000);
	writeByte(PRE_SCALE, prescale);
	writeByte(MODE1, 0x80);		//restart
	writeByte(MODE2, 0x0C);
	return;
}

void PCA9685::setPWM(int out_nr, uint16_t on_value, uint16_t off_value)
{
	writeWord(LED0_ON_L + LED_MULTIPLYER*out_nr, on_value);
	writeWord(LED0_OFF_L + LED_MULTIPLYER*out_nr, off_value);
}

void PCA9685::setPWM(int out_nr, uint16_t value)
{
	//cout<<"setting PWM "<<out_nr<<" to: "<< value<<endl;
	setPWM(out_nr, 0, value);
}

uint16_t PCA9685::getPWM(uint8_t out_nr)
{
	uint16_t value;
	value = wiringPiI2CReadReg8(fd, LED0_OFF_H + LED_MULTIPLYER*out_nr);
	value <<= 8;
	value += wiringPiI2CReadReg8(fd, LED0_OFF_L + LED_MULTIPLYER*out_nr);
	value &= 0x0fff;
	return value;
}

uint8_t PCA9685::readReg(uint8_t reg)
{
	return wiringPiI2CReadReg8(fd, reg);
}