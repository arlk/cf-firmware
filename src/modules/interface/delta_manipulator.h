/*
 *
 *                    .oo .oPYo.  .oPYo. o
 *                   .P 8 8    8  8   `8 8
 *                  .P  8 8      o8YooP' 8
 *                 oPooo8 8       8   `b 8
 *                .P    8 8    8  8    8 8
 *               .P     8 `YooP'  8    8 8oooo
 *               ..:::::..:.....::..:::........
 *               ::::::::::::::::::::::::::::::
 *               ::::::::::::::::::::::::::::::
 *               Advanced Controls Research Lab
 *
 * Copyright Advanced Controls Research Laboratory 2017
 * Univesity of Illinois at Urbana-Champaign
 * Visit us at http://naira.mechse.illinois.edu/
 *
 * Authored by Robert Mitchell Jones
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * delta_manipulator.h - servo motor controller for delta manipulator
 * */
#ifndef DELTA_MANIPULATOR_H_
#define DELTA_MANIPULATOR_H_

#include <stdbool.h>
#include <stdint.h>

#define RATE_MANIPULATOR_LOOP 100  // 100 Hz
#define BAUD_RATE 57142

#define DELTA_BROADCAST_ID 0xfe
#define DELTA_STATUS_PACKET_LENGTH 0x06
// Instruction set
#define DELTA_PING 0x01
#define DELTA_READ_DATA 0x02
#define DELTA_WRITE_DATA 0x03
#define DELTA_REG_WRITE 0x04
#define DELTA_ACTION 0x05
#define DELTA_RESET 0x06
#define DELTA_SYNC_WRITE 0x83
// Control table
#define DELTA_ADDRESS_PRESENT_TEMPERATURE 0x2B
#define DELTA_ADDRESS_GOAL_POSITION 0x1E
#define DELTA_ADDRESS_LED 0x19
#define DELTA_ADDRESS_MOVING_SPEED 0x20
#define DELTA_ADDRESS_RETURN_DELAY_TIME 0x05
#define DELTA_ADDRESS_MAX_TORQUE 0x0E
#define DELTA_ADDRESS_TORQUE_ENABLE 0x18
#define DELTA_ADDRESS_CW_ANGLE_LIMIT 0x06
#define DELTA_ADDRESS_CCW_ANGLE_LIMIT 0x08
// Message length
#define DELTA_STD_LEN 0x04
#define DELTA_SINGLE_MSG_LEN 0x04
#define DELTA_DOUBLE_MSG_LEN 0x05

void manipulatorInit(void);

void calcChksum(uint8_t* msg);

void dynamixelEnableTorque(uint8_t motorID, uint8_t torqueState);

void dynamixelSetLED(uint8_t motorID, uint8_t ledState);

void dynamixelMovePosn(uint8_t motorID, uint16_t position);

#endif /* DELTA_MANIPULATOR_H_ */
