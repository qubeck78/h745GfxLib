/*
 * console.h
 *
 *  Created on: Jan 3, 2025
 *      Author: qubec
 */

#ifndef GFXLIB_CONSOLE_H_
#define GFXLIB_CONSOLE_H_

#include "stm32h7xx_hal.h"

typedef struct _console_t
{
   uint16_t  width;
   uint16_t  height;

   uint16_t  cursX;
   uint16_t  cursY;
   uint8_t   textAttributes;

   uint8_t  *textBuffer;
   uint16_t *frameBuffer;
   uint16_t  fbWidth;
   uint16_t  fbHeight;
   uint16_t  fbRowWidth;

}console_t;

#define color565( r, g, b ) ( uint16_t )((((uint16_t)b >> 3) & 31 ) | (((uint16_t)g & 252 ) << 3 ) | (((uint16_t)r & 248 ) << 8 ))


uint32_t conRedraw( console_t *con );
uint32_t conSetCursorPos( console_t *con, uint16_t cursX, uint16_t cursY );
uint32_t conCls( console_t *con );
uint32_t conScrollUp( console_t *con );
uint32_t conPrint( console_t *con, char *string );


#endif /* GFXLIB_CONSOLE_H_ */
