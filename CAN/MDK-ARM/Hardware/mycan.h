#ifndef _MYCAN_H
#define _MYCAN_H
#include "stm32f1xx_hal.h"

 extern CAN_TxHeaderTypeDef TxHeader;
 extern CAN_FilterTypeDef CAN_FilterStructure;

HAL_StatusTypeDef CAN_Send_Message(uint32_t id, uint8_t* data, uint8_t length, 
                                  uint32_t ide_type, uint32_t rtr_type);
void can_filter(void);
void can_recive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif 
