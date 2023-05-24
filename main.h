#include <stdio.h>
#include <stdlib.h>
//#include "stm32_adafruit_spi_usd.h"
//#include "stm32_adafruit_spi_lcd.h"


#define Bank1_PSRAM1_ADDR	((uint32_t)0x60000000)
#define ADC1_DR_Address  0x4001244C
#define SPI1_DR_Address  0x4001300C


#define FlashLastPageAddr    ((uint32_t)0x0800fc00)	   //F103C8: 0x0800fc00   //F103VG: 0x080ff800    //F103C6: 0x08007c00
#define FlashPageSize	  0x400

#define ModBusUartSpeed	  19200

#define BKPSize 10

ErrorStatus HSEStartUpStatus;

DMA_InitTypeDef DMA_InitStructure;

u16 ADC_DMA_buffer[2];

//--- конфигурация датчика -- часть интерфейсной структуры для обмена по модбасу ---------
struct corr_conf_t
  {
    u16 reserved;		     //0  пока не исп
    s16 press_i2c;			 //1	 давление
    s16 temper_i2c;			 //2	 температура
    u16 i2c_sens_error; 	 //3
    u16 i2c_success_count;	 //4	 число удачных измерений
    u16 OurMBAddress;		 //5	 адрес на модбасе
    s16 press_offset;		 //6     смещение нуля давления
    s16 rawtemper_i2c;		 //7	сырая температура
    s16 rawpress_i2c;		 //8	сырое давление
    s16 kf_press;		     //9	коэффициент коррекции давления
    s16 temper_base;		 //A  10d температура настройки датчика, базовая температура. град * 100
    u16 press_diap;			 //B  11d  диапазон сенсора на нормируемом давлении диапазона измерения
    s16 tcomp_offset;		 //C  12d значение термокомпенсации нуля
    s16 tcomp_diap;			 //D  13d значение термокомпенсации диапазона

    u16 ustav;               //E 14d уставка по давлению
    s16 error;               //F 15d ошибка регулятора
    u16 power;               //10  16d мощность
    s16 filter;              //11  17d фильтр накополение

    u16 PCA9534_0;               //12 18d регистр 0 только чтение (8 входных уровней PCA9534) в младшем байте
    u16 PCA9534_1;               //13 19d регистр 1 (8 выходных уровней PCA9534) в младшем байте
    u16 PCA9534_3;               //14 20d регистр 3 (8 бит направлений PCA9534) в младшем байте

    s16 temperature;         //15  21d   температура
    u16 rheostat;            //16  22d   положение резистора
//    u16 PCA9534_2;               //17  23d    регистр 2 только чтение (инверсия входов PCA9534) в младшем байте

    u16 WorkMode;            //17  23d   режим работы контролллера
    u16 LightTime;            //18  24d   время включения освещения
    u16 BlowTime;            //19  25d   время обработки
    u16 CheckTime;           //1a  26d   время от вкл мотора до начала проверки фильтра
    u16 FanSpeed;          //1b  27d  3b 59d скорость вентилятора в Гц
    u16 sw_i2c_connection;   //1c  28d

    u16 InFanPower;         //ld  29d задаваемая мощность вентилятора, %*100
    u16 FFMnumber;         //le  30d общее количество ФВМ

  };

struct vent_conf_t
  {
  u16     fan_on;            //40 вкл вентилятора
  u16     set_select;        //41 выбор уставки
  u16     con_alarm;         //42
  u16     fan_alarm;         //43
  u16     filter_alarm;	     //44
  u16     main_alarm;        //45
  u16     fan_overload;      //46
  u16     fan_power;         //47
  u16     flaw_set1;         //48 уставка 1
  u16     flaw_set2;         //49 уставка 2
  s16     cur_flaw;          //4A скорость потока = K*sqrt(давление)  только для 1го канала
  u16     min_pres;          //4B минимальное давление на фильтре
  u16     max_pres;          //4C максимальное давление на фильтре
  u16     FanOverloadSet;    //4D
  u16     UV_on;             //4E
  u16     LightOn;           //4F
  u16     WorkHours;         //50
  u16     WorkHoursReset;    //51
  u16     UVHours;           //52
  u16     UVHoursReset;      //53
  u16     Kfactor;           //54
  u16     Pcoef;             //55
  u16     Icoef;             //56
  u16     ZoneNumber;        //57
  u16     FVMNumber;         //58
  u16     FanSpeed;          //59
  u16     UVSecondCounter;   //5A счётчик секунд работы УФ-лампы
  u16     Version;           //5B
  u16     HX711_temper;      //5C
  u16     HX711_press;       //5D
  u16     debug;             //5e debug
  u16     debug2;             //5f debug

  };

//--- интерфейсная структура -- для обмена по модбасу ---------
struct dev_conf_t
  {
	s16 devConfig[32];  //конфигурация регистров чтения-записи для модбаса (формат == структуре  corr_conf_t)
  };

struct dev_conf_t dev_conf[3];

//номера параметров (регистров) для записи во флеш (равны индексам двухбайтовых ячеек в структуре corr_conf_t)
#define CH2 (sizeof(dev_conf[0])/2)
//const uint8_t ParNumTable [] =	{5, 6, 9, 5+CH2, 6+CH2};
//const uint8_t ParNumTable [] =	{5, 6, 9, 0xa, 0xb, 6+CH2, };
const uint8_t ParNumTable [] =	{5, 6, 9, 0xa, 0xb, 6+CH2, 0x17, 0x18, 0x19, 0x1a, 0x1d, 0x1e};
const uint8_t ParNumTableSize = sizeof(ParNumTable)/sizeof(ParNumTable[0]);
const uint8_t ParNumTableBKP [] =	{0x5A, 9+CH2, 0x54, 0xb+CH2, 0x52, 0x4b, 0x4c, 0x48, 0x49};
const uint8_t ParNumTableBKPSize = sizeof(ParNumTableBKP)/sizeof(ParNumTableBKP[0]);

#undef CH2

struct corr_conf_t *CorrConf[2];
struct vent_conf_t *VentConf;

//struct corr_conf_t *CorrConf2;

u8 RTCounter;	     //счётчик обмена модбаса
u8 frame[220];	    //уарт буфер приёма/передачи

uint8_t GA_Adress;       //адрес нашего устройства на слейв-и2ц.

uint8_t TxBuffer[10];    //массив для передачи данных мастеру.
uint8_t Tx_Idx;
uint8_t RxBuffer[10];    //массив для приема данных от мастера.
uint8_t Rx_Idx;

uint32_t stop_count;
