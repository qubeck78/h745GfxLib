/*
 * console.c
 *
 *  Created on: Jan 3, 2025
 *      Author: qubec
 */

#include "console.h"

extern uint32_t _lcdFont;

static uint32_t conDrawLetter( console_t *con, uint16_t letterX, uint16_t letterY, uint8_t letter, uint16_t letterColor, uint16_t backgroundColor )
{
   uint16_t     x;
   uint16_t     y;
   uint8_t     *fontPtr;
   uint32_t     letterIdx;
   uint8_t      fontData;
   uint16_t    *pixelPtr;

   fontPtr = (uint8_t*) &_lcdFont;


   letterIdx = letter * 8;

   for( y = 0; y < 8; y++ )
   {
      fontData = fontPtr[letterIdx++];

      pixelPtr = &con->frameBuffer[ letterX * 8 + ( ( letterY * 8 ) + y ) * con->fbRowWidth ];

      for( x = 0; x < 8; x++ )
      {
         if( fontData & 0x80 )
         {
            *pixelPtr++ = letterColor;
         }
         else
         {
            *pixelPtr++ = backgroundColor;
         }

         fontData <<= 1;
      }
   }

   return 0;
}


uint32_t conRedraw( console_t *con )
{
   uint16_t     x;
   uint16_t     y;
   uint8_t      letter;
   uint8_t      attributes;
   uint8_t      r;
   uint8_t      g;
   uint8_t      b;
   uint16_t     letterColor;
   uint16_t     backgroundColor;

   uint8_t     *textDataPtr;

   if( !con )
   {
      return 1;
   }

   for( y = 0; y < con->height; y++ )
   {
      textDataPtr = &con->textBuffer[ y * con->width * 2 ];


      for( x = 0; x < con->width; x++ )
      {
         letter      = *textDataPtr++;
         attributes  = *textDataPtr++;

         //letter color

         r = 0;
         g = 0;
         b = 0;

         if( attributes & 8 )
         {
            //bright
            if( attributes & 1 )
            {
               r = 0xff;
            }

            if( attributes & 2 )
            {
               g = 0xff;
            }

            if( attributes & 4 )
            {
               b = 0xff;
            }

         }
         else
         {
            //dark
            if( attributes & 1 )
            {
               r = 0x80;
            }

            if( attributes & 2 )
            {
               g = 0x80;
            }

            if( attributes & 4 )
            {
               b = 0x80;
            }

         }

         letterColor = color565( r, g, b );


         //background color

         r = 0;
         g = 0;
         b = 0;

         attributes >>= 4;

         if( attributes & 8 )
         {
            //bright
            if( attributes & 1 )
            {
               r = 0xff;
            }

            if( attributes & 2 )
            {
               g = 0xff;
            }

            if( attributes & 4 )
            {
               b = 0xff;
            }

         }
         else
         {
            //dark
            if( attributes & 1 )
            {
               r = 0x80;
            }

            if( attributes & 2 )
            {
               g = 0x80;
            }

            if( attributes & 4 )
            {
               b = 0x80;
            }

         }

         backgroundColor = color565( r, g, b );


         conDrawLetter( con, x, y, letter, letterColor, backgroundColor );

      }
   }

   return 0;
}

uint32_t conSetCursorPos( console_t *con, uint16_t cursX, uint16_t cursY )
{
   if( !con )
   {
      return 1;
   }

   if( ( cursX < con->width ) && ( cursY < con->height ) )
   {

      con->cursX = cursX;
      con->cursY = cursY;

   }
   else
   {
      return 2;
   }

   return 0;
}

uint32_t conCls( console_t *con )
{
   uint32_t i;

   if( !con )
   {
      return 1;
   }


   for( i = 0; i < ( con->width * con->height * 2 ); i += 2 )
   {
      con->textBuffer[i]     = 0;
      con->textBuffer[i+1]   = con->textAttributes;
   }

   con->cursX = 0;
   con->cursY = 0;

   return 0;
}


uint32_t conScrollUp( console_t *con )
{
   uint32_t idx;
   uint32_t idx2;
   uint16_t x;
   uint16_t y;

   //scroll up
   if( !con )
   {
      return 1;
   }

   for( y = 0; y < con->height ; y++ )
   {
      //dest
      idx   = ( y * con->width * 2 );
      //src
      idx2  = ( ( y + 1 ) * con->width * 2 );

      if( y < con->height - 1 )
      {
         //scroll

         for( x = 0; x < con->width; x++ )
         {
            con->textBuffer[ idx++ ] = con->textBuffer[ idx2++ ];
            con->textBuffer[ idx++ ] = con->textBuffer[ idx2++ ];
         }

      }
      else
      {
         //clear last line
         for( x = 0; x < con->width; x++ )
         {
            con->textBuffer[ idx++ ] = 0;
            con->textBuffer[ idx++ ] = con->textAttributes;
         }
      }
   }

   return 0;

}

uint32_t conPrint( console_t *con, char *string )
{
   char     c;
   uint32_t idx;

   if( !con )
   {
      return 1;
   }

   while( ( c = *string++ ) )
   {
      idx = ( con->cursX * 2 ) + ( con->cursY * con->width * 2 );


      if( c >= 32 )
      {
         con->textBuffer[idx++] = c;
         con->textBuffer[idx++] = con->textAttributes;

         con->cursX++;

         if( con->cursX >= con->width )
         {
            con->cursX = 0;
            con->cursY++;
         }
      }
      else if( c == 10 )
      {
         con->cursX = 0;
         con->cursY++;
      }


      if( con->cursY >= con->height )
      {
         con->cursY = con->height - 1;

         conScrollUp( con );

      }
   }

   return 0;
}
