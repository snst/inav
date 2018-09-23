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

#define TARGET_BOARD_IDENTIFIER "AFNA" // AFroNAze - NAZE might be considered misleading on Naze clones like the flip32.

#define LED0                    PB3
#define LED1                    PB4

#define BEEPER                  PA12

#define INVERTER_PIN_UART2      PB2 // PB2 (BOOT1) abused as inverter select GPIO

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL
#define MAG_INT_EXTI            PC14
#define USE_MAG_DATA_READY_SIGNAL

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define NAZE_SPI_BUS            BUS_SPI2
#define NAZE_SPI_CS_GPIO        GPIOB
#define NAZE_SPI_CS_PIN         PB12
#define NAZE_CS_GPIO_CLK_PERIPHERAL RCC_APB2Periph_GPIOB

#define USE_UART1
#define USE_UART2
//#define USE_SOFTSERIAL1

#define USE_HARDWARE_REVISION_DETECTION
//#define USE_SOFTSERIAL2

#define M25P16_CS_GPIO          NAZE_SPI_CS_GPIO
#define M25P16_CS_PIN           NAZE_SPI_CS_PIN
#define M25P16_SPI_BUS          NAZE_SPI_BUS

#define SERIAL_PORT_COUNT       2       // UART1, UART2, SS1, SS2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_I2C
#define USE_I2C_DEVICE_2


#define USE_GYRO
#define USE_GYRO_MPU6050
#define MPU6050_I2C_BUS         I2CDEV_2
#define USE_GYRO_MPU6500

#define GYRO_MPU6050_ALIGN      CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define USE_ACC
#define USE_ACC_MPU6050

#define ACC_MPU6050_ALIGN       CW0_DEG
#define ACC_MPU6500_ALIGN       CW0_DEG

#define USE_BARO
#define USE_BARO_MS5611 
#define MS5611_I2C_BUS          I2CDEV_2

#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG
#define MAG_I2C_BUS             I2CDEV_2

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04_I2C
#define HCSR04_I2C_BUS          I2CDEV_2

#define SOFTSERIAL_1_RX_PIN     PA6
#define SOFTSERIAL_1_TX_PIN     PA7
#define SOFTSERIAL_2_RX_PIN     PB0
#define SOFTSERIAL_2_TX_PIN     PB1


// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67


#define USE_ADC
#define ADC_CHANNEL_1_PIN               PB1
#define ADC_CHANNEL_2_PIN               PA4
#define ADC_CHANNEL_3_PIN               PA1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

//#define USE_SPEKTRUM_BIND
//#define BIND_PIN                PA3

//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_MOTOR_COUNT      4

#define DEFAULT_FEATURES        (FEATURE_VBAT)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )

#define USE_ASYNC_GYRO_PROCESSING
#define USE_DEBUG_TRACE
#define USE_GYRO_NOTCH_1
#define USE_GYRO_NOTCH_2
#define USE_DTERM_NOTCH
#define USE_ACC_NOTCH
#define USE_GPS_PROTO_I2C_NAV
#define GPS_I2C_INSTANCE I2CDEV_2

#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION

#undef USE_RX_PWM
#undef USE_RX_PPM
#define USE_SERIAL_RX
#define USE_SERIALRX_SPEKTRUM   // Cheap and fairly common protocol
#undef USE_SERIALRX_SBUS       // Very common protocol
#undef USE_SERIALRX_IBUS       // Cheap FlySky & Turnigy receivers
#undef USE_SERIALRX_FPORT


#undef USE_SERVOS
#undef USE_BLACKBOX
#undef USE_GPS_PROTO_UBLOX
#undef USE_TELEMETRY
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_FRSKY
#undef USE_FLM_TURN_ASSIST