/*
 * modbus.h
 *
 *  Created on: Jul 15, 2022
 *      Author: kyleh
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "stm32f4xx_hal.h"


int8_t handleComms(UART_HandleTypeDef *huart, uint16_t *registers);

void modBusSend(UART_HandleTypeDef *huart, uint8_t len);
#endif /* INC_MODBUS_H_ */
