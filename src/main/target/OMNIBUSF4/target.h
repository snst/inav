/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef OMNIBUSF4PRO_LEDSTRIPM5
#define OMNIBUSF4PRO
#endif
#ifdef OMNIBUSF4PRO
#define TARGET_BOARD_IDENTIFIER "OBSD"
#elif defined(OMNIBUSF4V3)
#define TARGET_BOARD_IDENTIFIER "OB43"
#elif defined(DYSF4PRO)
#define TARGET_BOARD_IDENTIFIER "DYS4"
#else
#define TARGET_BOARD_IDENTIFIER "OBF4"
#endif

#if defined(DYSF4PRO)
#define USBD_PRODUCT_STRING "DysF4Pro"
#else
#define USBD_PRODUCT_STRING     "Omnibus F4"
#endif

#define LED0                    PB5

#define BEEPER                  PB4
#define BEEPER_INVERTED

#if defined(OMNIBUSF4V3)
  #define INVERTER_PIN_UART6      PC8
#else
  #define INVERTER_PIN_UART1      PC0 // PC0 has never been used as inverter control on genuine OMNIBUS F4 variants, but leave it as is since some clones actually implement it.
#endif

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2_SHARES_UART3

#define UG2864_I2C_BUS BUS_I2C2

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_ACC

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_GYRO_MPU6000
  #define GYRO_MPU6000_ALIGN      CW270_DEG

  #define USE_ACC_MPU6000
  #define ACC_MPU6000_ALIGN       CW270_DEG
#else
  #define USE_GYRO_MPU6000
  #define GYRO_MPU6000_ALIGN      CW180_DEG

  #define USE_ACC_MPU6000
  #define ACC_MPU6000_ALIGN       CW180_DEG
#endif

// Support for OMNIBUS F4 PRO CORNER - it has ICM20608 instead of MPU6000
#if defined (OMNIBUSF4PRO) || defined(OMNIBUSF4V3) || defined(OMNIBUSF4PRO_LEDSTRIPM5)
  #define MPU6500_CS_PIN          MPU6000_CS_PIN
  #define MPU6500_SPI_BUS         MPU6000_SPI_BUS

  #define USE_GYRO_MPU6500
  #define GYRO_MPU6500_ALIGN      GYRO_MPU6000_ALIGN

  #define USE_ACC_MPU6500
  #define ACC_MPU6500_ALIGN       ACC_MPU6000_ALIGN
#endif

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define MAG_HMC5883_ALIGN       CW90_DEG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_MAG3110

#define USE_BARO

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_BARO_BMP280
  #define BMP280_SPI_BUS        BUS_SPI3
  #define BMP280_CS_PIN         PB3 // v1
#else
  #define BARO_I2C_BUS          BUS_I2C2
  #define USE_BARO_BMP085
  #define USE_BARO_BMP280
  #define USE_BARO_MS5611
#endif

#define USE_PITOT_MS4525
#define PITOT_I2C_BUS           BUS_I2C2

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C2
#define USE_RANGEFINDER_HCSR04_I2C

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#if defined(OMNIBUSF4V3)
#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6
#else
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PC8
#define SOFTSERIAL_1_TX_PIN     PC9

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3, USART6, SOFTSERIAL1
#endif

#define USE_SPI

#define USE_SPI_DEVICE_1

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_SPI_DEVICE_2
  #define SPI2_NSS_PIN          PB12
  #define SPI2_SCK_PIN          PB13
  #define SPI2_MISO_PIN         PB14
  #define SPI2_MOSI_PIN         PB15
#endif

#define USE_SPI_DEVICE_3
#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define SPI3_NSS_PIN          PA15
#else
  #define SPI3_NSS_PIN          PB3
#endif
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PA15

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
  #define USE_SDCARD
  #define USE_SDCARD_SPI2

  #define SDCARD_DETECT_INVERTED
  #define SDCARD_DETECT_PIN               PB7
  #define SDCARD_SPI_INSTANCE             SPI2
  #define SDCARD_SPI_CS_PIN               SPI2_NSS_PIN

  #define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
  #define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
  #define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
  #define SDCARD_DMA_CHANNEL                  DMA_Channel_0
#else
  #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
  #define M25P16_CS_PIN           SPI3_NSS_PIN
  #define M25P16_SPI_BUS          BUS_SPI3
  #define USE_FLASHFS
  #define USE_FLASH_M25P16
#endif

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2

#ifdef DYSF4PRO
    #define ADC_CHANNEL_3_PIN               PC3
#else
    #define ADC_CHANNEL_3_PIN               PA0
#endif

#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define USE_LED_STRIP
#if (defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)) && !defined(OMNIBUSF4PRO_LEDSTRIPM5)
#   define WS2811_PIN                      PB6
#   define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST0_HANDLER
#   define WS2811_DMA_STREAM               DMA1_Stream0
#   define WS2811_DMA_CHANNEL              DMA_Channel_2
#else
#   define WS2811_PIN                      PA1
#   define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST4_HANDLER
#   define WS2811_DMA_STREAM               DMA1_Stream4
#   define WS2811_DMA_CHANNEL              DMA_Channel_6
#endif

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define DISABLE_RX_PWM_FEATURE
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_OSD)

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PB11 // USART3 RX

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6
#define TARGET_MOTOR_COUNT      6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
#define USABLE_TIMER_CHANNEL_COUNT 13
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(4) | TIM_N(8) | TIM_N(9) | TIM_N(10) )
#else
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) )
#endif

#ifdef OMNIBUSF4PRO
#define CURRENT_METER_SCALE   265
#endif
