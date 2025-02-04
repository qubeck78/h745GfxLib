/*
 * bsp.c
 *
 *  Created on: Jan 31, 2025
 *      Author: qubeck
 */

#include "bsp.h"
#include "cmsis_os.h"

#include <stdint.h>
#include <stddef.h>  // for size_t
#include <unistd.h>  // for STDERR_FILENO

#include "gfFont.h"
#include "osAlloc.h"

extern uint32_t _lcdFont;


uint32_t    randomSeed = 3242323459;

tgfTextOverlay con;
tgfFont        lcdFont;


uint32_t bspInit()
{

   osAllocInit();
   osAllocAddNode( 0, ( void* )_SYSTEM_MEMORY_BASE, _SYSTEM_MEMORY_SIZE, OS_ALLOC_MEMF_CHIP );

   lcdFont.backgroundColor = 0;
   lcdFont.charColor       = 0xffff;
   lcdFont.type            = GF_FONT_TYPE_MONO_BITMAP_FIXED_WIDTH;
   lcdFont.firstChar       = 0;
   lcdFont.lastChar        = 255;
   lcdFont.width           = 8;
   lcdFont.height          = 8;

   lcdFont.charBuffer   = &_lcdFont;


   //connect gfxlib con to software text overlay
   con.type                = GF_TEXT_OVERLAY_TYPE_SOFTWARE;
   con.flags               = 0;
   con.width               = 60;               //clear whole buffer
   con.height              = 34;
   con.cursX               = 0;
   con.cursY               = 0;
   con.textAttributes      = 0x0f;
   con.font                = &lcdFont;
   con.textBuffer          = (uint8_t*) osAlloc( 60 * 34 * 2, OS_ALLOC_MEMF_CHIP );

   con.textAttributes   = 0x0f;
   toCls( &con );
   con.textAttributes   = 0x8f;


   randomSeed += getTicks();

   return 0;
}

uint32_t randomNumber()
{
    uint32_t r = randomSeed;

    r ^= r << 13;
    r ^= r >> 17;
    r ^= r << 5;

    randomSeed = r;

    return r;

/*
    //https://github.com/cmcqueen/simplerandom/blob/main/c/lecuyer/lfsr88.c
   uint32_t b;

    b = (((s1 << 13) ^ s1) >> 19);
    s1 = (((s1 & 4294967294) << 12) ^ b);
    b = (((s2 << 2) ^ s2) >> 25);
    s2 = (((s2 & 4294967288) << 4) ^ b);
    b = (((s3 << 3) ^ s3) >> 11);
    s3 = (((s3 & 4294967280) << 17) ^ b);
    return (s1 ^ s2 ^ s3);
    */
}

void hexDigit(char *string,char digit)
{
    digit &= 0x0f;

    if( digit<10 )
    {
        string[0] = digit + '0';
        string[1] = 0;
    }
    else
    {
        string[0] = digit + 'a' - 10;
        string[1] = 0;
    }
}

void itoaHex2Digits( uint32_t value, char* str )
{
    hexDigit(&str[0], ( value >> 4 ) & 0x0f );
    hexDigit(&str[1], ( value ) & 0x0f );
}

void itoaHex4Digits( uint32_t value, char* str )
{
    hexDigit(&str[4], ( value >> 12 ) & 0x0f );
    hexDigit(&str[5], ( value >> 8 ) & 0x0f );

    hexDigit(&str[6], ( value >> 4) & 0x0f );
    hexDigit(&str[7], ( value ) & 0x0f );
}


void itoaHex8Digits( uint32_t value, char* str )
{
    hexDigit(&str[0], ( value >> 28 ) & 0x0f );
    hexDigit(&str[1], ( value >> 24 ) & 0x0f );

    hexDigit(&str[2], ( value >> 20 ) & 0x0f );
    hexDigit(&str[3], ( value >> 16 ) & 0x0f );

    hexDigit(&str[4], ( value >> 12 ) & 0x0f );
    hexDigit(&str[5], ( value >> 8 ) & 0x0f );

    hexDigit(&str[6], ( value >> 4) & 0x0f );
    hexDigit(&str[7], ( value ) & 0x0f );
}

uint32_t getTicks()
{
   return HAL_GetTick();
}

void delayMs( uint32_t delay )
{
   osDelay( delay );

}

uint32_t setVideoMode( uint32_t videoMode )
{
   return 1;
}


