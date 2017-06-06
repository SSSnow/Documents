// ************************************************************************************************************
// 气压计    MPC2520 支持文件 
// 支持器件  :MPC2520
// 移植及注释:Snow 2017-01
// ************************************************************************************************************
#include "MPC2520.h"
#include "delay.h"	
#include "device.h"
#include "driver.h"

static void MPC2520_i2c_init(void);
static uint8_t MPC2520_i2c_write_byte(unsigned char deviceAddr, unsigned char regAddr);
static uint8_t MPC2520_i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer);
static uint8_t MPC2520_reset(void);
static unsigned char MPC2520Detect(void);
static unsigned short MPC2520_prom(char coef_num);
static signed char MPC2520_crc(unsigned short *prom);
static uint8_t MPC2520_read_adc(unsigned int *ADC_Val);
static uint8_t MPC2520_start_ut(void);
static uint8_t MPC2520_get_ut(void);
static uint8_t MPC2520_start_up(void);
static uint8_t MPC2520_get_up(void);
static void MPC2520_calculate(int *pressure, int *temperature);
static int MPC2520_init(void);
static int MPC2520_ioctrl(unsigned char cmd, void* arg);

static unsigned int MPC2520_ut = 0;  // static result of temperature measurement
static unsigned int MPC2520_up = 0;  // static result of pressure measurement
static unsigned short MPC2520_c[PROM_NB];  // on-chip ROM ??????
static unsigned char MPC2520_osr = CMD_ADC_4096;

static unsigned int  MPC2520ID = 0;

static DEV MPC2520 = {
	.name = "MPC2520",
	.devDrv = {
		.init = MPC2520_init,
		.ioctrl = MPC2520_ioctrl
	}
};

unsigned int MPC2520_getID(void)
{
	return MPC2520ID;
}
unsigned int MPC2520_register(void)
{
	MPC2520ID = register_driver(&MPC2520.devDrv);
	return  MPC2520ID;
}

static void  MPC2520_i2c_init(void)
{
//	REG_CCFG_SYSIO = (1<<5) | (1<<4);
//	Pin PIN_I2C1_SDA = {PIO_PB4A_TWD1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_OPENDRAIN}; 
//	Pin PIN_I2C1_SCL = {PIO_PB5A_TWCK1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_OPENDRAIN}; 

//	PMC_EnablePeripheral(ID_TWIHS1);
//	PIO_Configure(&PIN_I2C1_SDA, 1);
//	PIO_Configure(&PIN_I2C1_SCL, 1);
//	i2c_init(TWIHS1, 400000, MCK);
//	delay_ms(10);
}

static uint8_t MPC2520_i2c_write_byte(unsigned char deviceAddr, unsigned char regAddr)
{
	unsigned char temp = regAddr;
	return i2c_write(TWIHS1, deviceAddr, &temp, 1);
}
static uint8_t MPC2520_i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer)
{
	return i2c_read(TWIHS1, slaveAddr, readBuffer, readNumber);
}
static  uint8_t MPC2520_reset(void)
{
	//Reset IIC Clock
	Pin PIN_SCL = {PIO_PB4, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT};	
	PIO_Configure(&PIN_SCL, 1);	
	PIO_Set(&PIN_SCL);
	for(uint8_t i = 0; i < 8; i ++)
	{
		PIO_Clear(&PIN_SCL);
		PIO_Set(&PIN_SCL);
	}

	REG_CCFG_SYSIO = (1 << 5) | (1 << 4);
	Pin PIN_I2C1_SDA = {PIO_PB4A_TWD1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_OPENDRAIN};
	Pin PIN_I2C1_SCL = {PIO_PB5A_TWCK1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_OPENDRAIN};

	PMC_EnablePeripheral(ID_TWIHS1);
	PIO_Configure(&PIN_I2C1_SDA, 1);
	PIO_Configure(&PIN_I2C1_SCL, 1);
	i2c_init(TWIHS1, 400000, MCK);
	
	return MPC2520_i2c_write_byte(MPC2520_ADDR, CMD_RESET);
}

//-------------------------------------------------------------
//                         ???MPC2520
//--------------------------------------------------------------
static unsigned char MPC2520Detect(void)
{
	MPC2520_reset();	//????
	delay_ms(20);
	//??16Byte PROM ????
	for (int i = 0; i < PROM_NB; i ++)MPC2520_c[i] = MPC2520_prom(i);

	if (MPC2520_crc(MPC2520_c) != 0) return 0;

	return 1;
}

//--------------------------------------------------------------
//                         ??2byte Prom
//--------------------------------------------------------------
static unsigned short MPC2520_prom(char coef_num)
{
	uint8_t Sta = 1;
    unsigned char rxbuf[2] = {0, 0};
	Sta &= MPC2520_i2c_write_byte(MPC2520_ADDR,CMD_PROM_RD + coef_num * 2);
    delay_ms(5);
	Sta &= MPC2520_i2c_read(MPC2520_ADDR,2,rxbuf); //2???PROM
    return rxbuf[0] << 8 | rxbuf[1];
}

//--------------------------------------------------------------
//                      ??PROM ?CRC?? 
//--------------------------------------------------------------
//static signed char MPC2520_crc(unsigned short *prom)
//{
//    int i, j;
//    unsigned int res = 0;
//    unsigned char zero = 1;
////    unsigned char crc = prom[7] & 0xFF;
//	unsigned int crc = prom[7];
//    prom[7] &= 0xFF00;
//    for (i = 0; i < 8; i++) {
//        if (prom[i] != 0)
//            zero = 0;
//    }
//    if (zero)
//        return -1;

//    for (i = 0; i < 16; i++) {
//        if (i & 0x01)
//            res ^= (unsigned short)((prom[i >> 1]) & 0x00FF);
//        else
//            res ^= (unsigned short)(prom[i >> 1] >> 8);
//        for (j = 8; j > 0; j--) {
//            if (res & 0x8000)
//			{
//				res = (res << 1) ^ 0x3000;
//			}
//			else
//			{
//				res <<= 1;
//			}
////                res ^= 0x1800;
////            res <<= 1;
//        }
//    }
//	res = (0x000F & (res >> 12));
////    prom[7] |= crc;
//	prom[7] = crc;
////    if (crc == ((res >> 12) & 0xF))
////        return 0;
//	if(crc == res) return 0;

//    return -1;
//}
static signed char MPC2520_crc(unsigned short *prom)
{
    int i, j;
    unsigned int res = 0;
    unsigned char zero = 1;
    unsigned char crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;
    for (i = 0; i < 8; i++) {
        if (prom[i] != 0)
            zero = 0;
    }
    if (zero)
        return -1;

    for (i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}
//--------------------------------------------------------------
//                 ?? 3Byte/24bit ADC?? 
//--------------------------------------------------------------
static uint8_t MPC2520_read_adc(unsigned int *ADC_Val)
{
	unsigned char rxbuf[3];	
	if(MPC2520_i2c_write_byte(MPC2520_ADDR, CMD_ADC_READ))
	{
		if(MPC2520_i2c_read(MPC2520_ADDR, 3, rxbuf))
		{
			*ADC_Val = (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
			return 1;
		}
	}
	return 0;
}
//--------------------------------------------------------------
//                 ?????? 
//--------------------------------------------------------------
static uint8_t MPC2520_start_ut(void)
{  
	return MPC2520_i2c_write_byte(MPC2520_ADDR,CMD_ADC_CONV + CMD_ADC_D2 + MPC2520_osr);// D2 (temperature) conversion start!
}

static uint8_t MPC2520_get_ut(void)
{
    return MPC2520_read_adc(&MPC2520_ut);
}
//--------------------------------------------------------------
//                ?????? 
//--------------------------------------------------------------
static uint8_t MPC2520_start_up(void)
{
	return MPC2520_i2c_write_byte(MPC2520_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MPC2520_osr);// D1 (pressure) conversion start!
}

static uint8_t MPC2520_get_up(void)
{
    return MPC2520_read_adc(&MPC2520_up);
}
//--------------------------------------------------------------
//                 ??Prom????
//--------------------------------------------------------------
static void MPC2520_calculate(int *pressure, int *temperature)
{
    unsigned int press;
    signed long long temp;
    signed long long delt;    
    int dT = (signed long long)MPC2520_ut - ((unsigned long long)MPC2520_c[5] * 256);
    signed long long off = ((signed long long)MPC2520_c[2] << 16) + (((signed long long)MPC2520_c[4] * dT) >> 7);
    signed long long sens = ((signed long long)MPC2520_c[1] << 15) + (((signed long long)MPC2520_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (signed long long)MPC2520_c[6]) >> 23);    

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((signed long long)MPC2520_up * sens ) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

static int MPC2520_init(void)
{
	MPC2520_i2c_init();
	return MPC2520Detect();
}

static int MPC2520_ioctrl(unsigned char cmd, void* arg)
{
	switch(cmd)
	{
		case MPC2520_IOCTRL_START_PRESSURE:
			return MPC2520_start_up();
		case MPC2520_IOCTRL_PRESSURE_READ:
			return MPC2520_get_up();
		case MPC2520_IOCTRL_START_TEMPERATURE:
			return MPC2520_start_ut();
		case MPC2520_IOCTRL_TEMPERATURE_READ:
			return MPC2520_get_ut();
		case MPC2520_IOCTRL_PRESSURE_CALCULATE:
		{
			int* pbuffer = arg;
			MPC2520_calculate(pbuffer, pbuffer + 1);
		}
		return 1;
		
		case  MPC2520_IOCTRL_RESET:
		return MPC2520_reset();	
		
		default: return 0;
	}
}

/*
 ??   
 #define BARO_FILTER_LEN 24
 static unsigned char state = 0; //??????????
 //buffer[0]:baroPressure  buffer[1]:baroTemperature
 int buffer[2]={0};
 static int baro_HistTab[BARO_FILTER_LEN]={0};
 int baro_SortTab[BARO_FILTER_LEN]={0};
 float baroPressureSum = 0;
 static unsigned char HistTab_index=0;
 if (state) 
 {
		ioctrl(MPC2520_getID(),MPC2520_IOCTRL_PRESSURE_READ,NULL);
		ioctrl(MPC2520_getID(),MPC2520_IOCTRL_START_TEMPERATURE,NULL);
    ioctrl(MPC2520_getID(),MPC2520_IOCTRL_PRESSURE_CALCULATE,buffer);
    baro_HistTab[HistTab_index] = buffer[0];
    HistTab_index ++;
		if(HistTab_index >= BARO_FILTER_LEN)
		{
			HistTab_index = 0;
		}
		
    memcpy(baro_SortTab,baro_HistTab, BARO_FILTER_LEN*4);
		
		sort(baro_SortTab,BARO_FILTER_LEN,0);
		
		int sum=0;
		for(unsigned char k=BARO_FILTER_LEN/4;k<(BARO_FILTER_LEN/4)*3;k++)
		{
				sum+= baro_SortTab[k];
		}
		
		baroPressureSum=(float)sum/(BARO_FILTER_LEN/2);
		
    state = 0;				
 } 	
 else 
 {
		ioctrl(MPC2520_getID(),MPC2520_IOCTRL_TEMPERATURE_READ,NULL);
    ioctrl(MPC2520_getID(),MPC2520_IOCTRL_START_PRESSURE,NULL);
    state = 1;     

 }

*/
