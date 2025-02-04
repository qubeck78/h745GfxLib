/*
 * bsp.h
 *
 *  Created on: Jan 31, 2025
 *      Author: qubeck
 */

#ifndef GFXLIB_BSP_H_
#define GFXLIB_BSP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"

//16MB
#define _SYSTEM_MEMORY_SIZE     ( 16777216 - 16)

#define _SYSTEM_MEMORY_BASE      0xd0000000


uint32_t    bspInit( void );
uint32_t    randomNumber( void );
void        itoaHex2Digits( uint32_t value, char* str );
void        itoaHex4Digits( uint32_t value, char* str );
void        itoaHex8Digits( uint32_t value, char* str );
uint32_t    getTicks( void );
void        delayMs( uint32_t delay );

uint32_t    setVideoMode( uint32_t videoMode );
void        reboot( void );


#define bspDCFlush()


static float inline ffMul( float a, float b )
{
    return a * b;
}

static float inline ffAdd( float a, float b )
{
    return a + b;
}

static float inline ffSub( float a, float b )
{
    return a - b;
}


static float inline ffDiv( float a, float b )
{
    return a / b;
}

#ifdef __cplusplus
}
#endif


#endif /* GFXLIB_BSP_H_ */
