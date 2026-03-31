#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <string.h>
#include "main.h"

uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);




#endif
