#ifndef MPC2520_C
#define MPC2520_C

#include "mpc2520.h"
#include "SoftwareI2C.h"

static struct mpc2520_t mpc2520;
static struct mpc2520_t *p_mpc2520;

void mpc2520_write(uint8 hwadr, uint8 regadr, uint8 val);
uint8 mpc2520_read(uint8 hwadr, uint8 regadr);
void mpc2520_get_calib_param(void);



/*****************************************************************************
 �� �� ��  : mpc2520_write
 ��������  : I2C �Ĵ���д���Ӻ���
 �������  : uint8 hwadr   Ӳ����ַ
             uint8 regadr  �Ĵ�����ַ
             uint8 val     ֵ
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_write(uint8 hwadr, uint8 regadr, uint8 val)
{
    SoftI2C_start();
    SoftI2C_write_byte(hwadr << 1);
    SoftI2C_write_byte(regadr);
    SoftI2C_write_byte(val);
    SoftI2C_stop();
}


/*****************************************************************************
 �� �� ��  : mpc2520_read
 ��������  : I2C �Ĵ�����ȡ�Ӻ���
 �������  : uint8 hwadr   Ӳ����ַ
             uint8 regadr  �Ĵ�����ַ
 �������  : 
 �� �� ֵ  : uint8 ����ֵ
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8 mpc2520_read(uint8 hwadr, uint8 regadr)
{
    uint8 val = 0;
    SoftI2C_start();
    SoftI2C_write_byte(hwadr << 1);
    SoftI2C_write_byte(regadr);
    SoftI2C_start();
    SoftI2C_write_byte((hwadr << 1)|0x01);
    val = SoftI2C_read_byte(1);
    SoftI2C_stop();
    return val;
}

/*****************************************************************************
 �� �� ��  : mpc2520_init
 ��������  : SPL06-01 ��ʼ������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_init(void)
{
    p_mpc2520 = &mpc2520; /* read Chip Id */
    p_mpc2520->i32rawPressure = 0;
    p_mpc2520->i32rawTemperature = 0;
    p_mpc2520->chip_id = 0x34;
    mpc2520_get_calib_param();
    // sampling rate = 1Hz; Pressure oversample = 2;
    mpc2520_rateset(PRESSURE_SENSOR,32, 8);   
    // sampling rate = 1Hz; Temperature oversample = 1; 
    mpc2520_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    
}

/*****************************************************************************
 �� �� ��  : mpc2520_rateset
 ��������  :  �����¶ȴ�������ÿ����������Լ���������
 �������  : uint8 u8OverSmpl  ��������         Maximal = 128
             uint8 u8SmplRate  ÿ���������(Hz) Maximal = 128
             uint8 iSensor     0: Pressure; 1: Temperature
 �������  : ��
 �� �� ֵ  : ��
 ���ú���  :
 ��������  :

 �޸���ʷ      :
  1.��    ��   : 2015��11��24��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        p_mpc2520->i32kP = i32kPkT;
        mpc2520_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = mpc2520_read(HW_ADR, 0x09);
            mpc2520_write(HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = mpc2520_read(HW_ADR, 0x09);
            mpc2520_write(HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        p_mpc2520->i32kT = i32kPkT;
        mpc2520_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = mpc2520_read(HW_ADR, 0x09);
            mpc2520_write(HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = mpc2520_read(HW_ADR, 0x09);
            mpc2520_write(HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

/*****************************************************************************
 �� �� ��  : mpc2520_get_calib_param
 ��������  : ��ȡУ׼����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_get_calib_param(void)
{
    uint32 h;
    uint32 m;
    uint32 l;
    h =  mpc2520_read(HW_ADR, 0x10);
    l  =  mpc2520_read(HW_ADR, 0x11);
    p_mpc2520->calib_param.c0 = (int16)h<<4 | l>>4;
    p_mpc2520->calib_param.c0 = (p_mpc2520->calib_param.c0&0x0800)?(0xF000|p_mpc2520->calib_param.c0):p_mpc2520->calib_param.c0;
    h =  mpc2520_read(HW_ADR, 0x11);
    l  =  mpc2520_read(HW_ADR, 0x12);
    p_mpc2520->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    p_mpc2520->calib_param.c1 = (p_mpc2520->calib_param.c1&0x0800)?(0xF000|p_mpc2520->calib_param.c1):p_mpc2520->calib_param.c1;
    h =  mpc2520_read(HW_ADR, 0x13);
    m =  mpc2520_read(HW_ADR, 0x14);
    l =  mpc2520_read(HW_ADR, 0x15);
    p_mpc2520->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    p_mpc2520->calib_param.c00 = (p_mpc2520->calib_param.c00&0x080000)?(0xFFF00000|p_mpc2520->calib_param.c00):p_mpc2520->calib_param.c00;
    h =  mpc2520_read(HW_ADR, 0x15);
    m =  mpc2520_read(HW_ADR, 0x16);
    l =  mpc2520_read(HW_ADR, 0x17);
    p_mpc2520->calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
    p_mpc2520->calib_param.c10 = (p_mpc2520->calib_param.c10&0x080000)?(0xFFF00000|p_mpc2520->calib_param.c10):p_mpc2520->calib_param.c10;
    h =  mpc2520_read(HW_ADR, 0x18);
    l  =  mpc2520_read(HW_ADR, 0x19);
    p_mpc2520->calib_param.c01 = (int16)h<<8 | l;
    h =  mpc2520_read(HW_ADR, 0x1A);
    l  =  mpc2520_read(HW_ADR, 0x1B);
    p_mpc2520->calib_param.c11 = (int16)h<<8 | l;
    h =  mpc2520_read(HW_ADR, 0x1C);
    l  =  mpc2520_read(HW_ADR, 0x1D);
    p_mpc2520->calib_param.c20 = (int16)h<<8 | l;
    h =  mpc2520_read(HW_ADR, 0x1E);
    l  =  mpc2520_read(HW_ADR, 0x1F);
    p_mpc2520->calib_param.c21 = (int16)h<<8 | l;
    h =  mpc2520_read(HW_ADR, 0x20);
    l  =  mpc2520_read(HW_ADR, 0x21);
    p_mpc2520->calib_param.c30 = (int16)h<<8 | l;
}


/*****************************************************************************
 �� �� ��  : mpc2520_start_temperature
 ��������  : ����һ���¶Ȳ���
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_start_temperature(void)
{
    mpc2520_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 �� �� ��  : mpc2520_start_pressure
 ��������  : ����һ��ѹ��ֵ����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_start_pressure(void)
{
    mpc2520_write(HW_ADR, 0x08, 0x01);
}

/*****************************************************************************
 �� �� ��  : mpc2520_start_continuous
 ��������  : Select node for the continuously measurement
 �������  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��25��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_start_continuous(uint8 mode)
{
    mpc2520_write(HW_ADR, 0x08, mode+4);
}

void mpc2520_stop(void)
{
    mpc2520_write(HW_ADR, 0x08, 0);
}


/*****************************************************************************
 �� �� ��  : mpc2520_get_raw_temp
 ��������  : ��ȡ�¶ȵ�ԭʼֵ����ת����32Bits����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_get_raw_temp(void)
{
    uint8 h,m,l;
    SoftI2C_start();
    SoftI2C_write_byte(HW_ADR << 1);
    SoftI2C_write_byte(0x03);
    SoftI2C_start();
    SoftI2C_write_byte((HW_ADR << 1)|0x01);
    h = SoftI2C_read_byte(0);
    m = SoftI2C_read_byte(0);
    l = SoftI2C_read_byte(1);
    SoftI2C_stop();
    p_mpc2520->i32rawTemperature = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_mpc2520->i32rawTemperature= (p_mpc2520->i32rawTemperature&0x800000) ? (0xFF000000|p_mpc2520->i32rawTemperature) : p_mpc2520->i32rawTemperature;
}

/*****************************************************************************
 �� �� ��  : mpc2520_get_raw_pressure
 ��������  : ��ȡѹ��ԭʼֵ����ת����32bits����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
void mpc2520_get_raw_pressure(void)
{
    uint8 h,m,l;
    SoftI2C_start();
    SoftI2C_write_byte(HW_ADR << 1);
    SoftI2C_write_byte(0x00);
    SoftI2C_start();
    SoftI2C_write_byte((HW_ADR << 1)|0x01);
    h = SoftI2C_read_byte(0);
    m = SoftI2C_read_byte(0);
    l = SoftI2C_read_byte(1);
    SoftI2C_stop();
    
    p_mpc2520->i32rawPressure = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_mpc2520->i32rawPressure= (p_mpc2520->i32rawPressure&0x800000) ? (0xFF000000|p_mpc2520->i32rawPressure) : p_mpc2520->i32rawPressure;
}


/*****************************************************************************
 �� �� ��  : mpc2520_get_temperature
 ��������  : �ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼����¶�ֵ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
float mpc2520_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_mpc2520->i32rawTemperature / (float)p_mpc2520->i32kT;
    fTCompensate =  p_mpc2520->calib_param.c0 * 0.5 + p_mpc2520->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 �� �� ��  : mpc2520_get_pressure
 ��������  : �ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼���ѹ��ֵ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : jason.li
    �޸�����   : �����ɺ���

*****************************************************************************/
float mpc2520_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_mpc2520->i32rawTemperature / (float)p_mpc2520->i32kT;
    fPsc = p_mpc2520->i32rawPressure / (float)p_mpc2520->i32kP;
    qua2 = p_mpc2520->calib_param.c10 + fPsc * (p_mpc2520->calib_param.c20 + fPsc* p_mpc2520->calib_param.c30);
    qua3 = fTsc * fPsc * (p_mpc2520->calib_param.c11 + fPsc * p_mpc2520->calib_param.c21);

    fPCompensate = p_mpc2520->calib_param.c00 + fPsc * qua2 + fTsc * p_mpc2520->calib_param.c01 + qua3;
    return fPCompensate;
}

#endif
