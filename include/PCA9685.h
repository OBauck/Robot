
#ifndef PCA9685_H_
#define PCA9685_H_

#define LED0 0x6 //LED0 start register
#define LED0_ON_L 0x6 //LED0 output and brightness control byte 0
#define LED0_ON_H 0x7 //LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8 //LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9 //LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4 // For the other 15 channels

#define MODE1 0x00 //Mode register 1
#define MODE2 0x01 //Mode register 2
#define SUBADR1 0x02 //I2C-bus subaddress 1
#define SUBADR2 0x03 //I2C-bus subaddress 2
#define SUBADR3 0x04 //I2C-bus subaddress 3
#define ALLCALLADR 0x05 //LED All Call I2C-bus address

#define PRE_SCALE 0xFE //prescaler for output frequency
#define CLOCK 25000000UL

#include <inttypes.h>

class PCA9685
{
	public:
	PCA9685(int, int);
	void reset();
	void writeByte(uint8_t, uint8_t);
	void writeWord(uint8_t, uint16_t);
	void setPWMFreq(int);
	void setPWM(int, uint16_t, uint16_t);
	void setPWM(int, uint16_t);
	uint16_t getPWM(uint8_t);
	uint8_t readReg(uint8_t);
	
	private:
	int fd;
};

#endif
