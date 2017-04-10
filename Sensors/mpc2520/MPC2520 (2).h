// ************************************************************************************************************
// I2C Barometer MPC2520
// ************************************************************************************************************

#ifndef MPC2520_H
#define  MPC2520_H
//#include "hal_board_cfg.h"
#include "platform.h"

#define HW_ADR 	0x77			  
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


struct mpc2520_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

struct mpc2520_t {	
    struct mpc2520_calib_param_t calib_param;/**<calibration data*/	
    uint8_t chip_id; /**<chip id*/	
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;    
    int32_t i32kT;
};

void mpc2520_init(void);
void mpc2520_rateset(uint8_t iSensor, uint8_t u8OverSmpl, uint8_t u8SmplRate);
void mpc2520_start_temperature(void);
void mpc2520_start_pressure(void);
void mpc2520_start_continuous(uint8_t mode);
void mpc2520_get_raw_temp(void);
void mpc2520_get_raw_pressure(void);
float mpc2520_get_temperature(void);
float mpc2520_get_pressure(void);

#endif