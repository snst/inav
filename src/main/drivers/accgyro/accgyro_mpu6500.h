/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define MPU6500_BIT_RESET                   (0x80)
#define MPU6500_BIT_INT_ANYRD_2CLEAR        (1 << 4)
#define MPU6500_BIT_BYPASS_EN               (1 << 0)
#define MPU6500_BIT_I2C_IF_DIS              (1 << 4)
#define MPU6500_BIT_RAW_RDY_EN              (0x01)


bool mpu6500AccDetect(accDev_t *acc);
bool mpu6500GyroDetect(gyroDev_t *gyro);
