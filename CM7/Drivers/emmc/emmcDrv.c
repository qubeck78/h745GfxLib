/*
 * emmcDrv.c
 *
 *  Created on: Jan 3, 2025
 *      Author: qubec
 */

#include "emmcDrv.h"
#include "ff_gen_drv.h"


MMC_HandleTypeDef *hmmc;

static DSTATUS emmcCheckStatus( BYTE lun );

DSTATUS emmcInitialize( BYTE lun );
DSTATUS emmcStatus( BYTE lun );
DRESULT emmcRead( BYTE lun, BYTE *buf, DWORD sector, UINT count );

#if _USE_WRITE == 1

DRESULT emmcWrite( BYTE lun, const BYTE *buf, DWORD sector, UINT count );

#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1

DRESULT emmcIoctl( BYTE lun, BYTE cmd, void *buf );

#endif  /* _USE_IOCTL == 1 */


const Diskio_drvTypeDef  emmcDriverStruct =
{
   emmcInitialize,
   emmcStatus,
   emmcRead,

   #if _USE_WRITE == 1

   emmcWrite,

   #endif /* _USE_WRITE == 1 */

   #if _USE_IOCTL == 1

   emmcIoctl,

   #endif /* _USE_IOCTL == 1 */

};

static DSTATUS emmcCheckStatus( BYTE lun )
{
   HAL_MMC_CardStateTypeDef cardState;

   cardState = HAL_MMC_GetCardState( hmmc );

   return( ( cardState == HAL_MMC_CARD_TRANSFER) ? MMC_TRANSFER_OK : MMC_TRANSFER_BUSY );
}

uint32_t emmcDrvInit( MMC_HandleTypeDef *phmmc )
{

   hmmc = phmmc;

   if( !phmmc )
   {
      return 1;
   }

   return 0;
}

DSTATUS emmcInitialize( BYTE lun )
{


   return 0;
}

DSTATUS emmcStatus( BYTE lun )
{
   return emmcCheckStatus( lun );
}

DRESULT emmcRead( BYTE lun, BYTE *buf, DWORD sector, UINT count )
{
   DRESULT res = RES_ERROR;
   uint32_t timeout = MMC_READ_TIMEOUT * count;

   if( HAL_MMC_ReadBlocks( hmmc, (uint8_t*)buf, (uint32_t) (sector), count, timeout ) == HAL_OK )
   {

/*     while(BSP_MMC_GetCardState(0) != BSP_ERROR_NONE)
     {
     }*/
       res = RES_OK;
   }
   else
   {
     res = RES_NOTRDY;
   }

   return res;
}


#if _USE_WRITE == 1

DRESULT emmcWrite( BYTE lun, const BYTE *buf, DWORD sector, UINT count )
{
   DRESULT res = RES_ERROR;
   uint32_t timeout = MMC_WRITE_TIMEOUT * count;
   HAL_StatusTypeDef halRv;

   halRv = HAL_MMC_WriteBlocks( hmmc, (uint8_t*)buf, (uint32_t)(sector), count, timeout );
   if( halRv == HAL_OK )
   {
      /*while(BSP_MMC_GetCardState(0) != BSP_ERROR_NONE)
      {
      }*/
      res = RES_OK;
   }

   return res;
}


#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1

DRESULT emmcIoctl( BYTE lun, BYTE cmd, void *buf )
{
   DRESULT                 res = RES_ERROR;
   HAL_MMC_CardInfoTypeDef halCardInfo;


   switch( cmd )
   {
      /* Make sure that no pending write process */
      case CTRL_SYNC :
        res = RES_OK;
        break;

      /* Get number of sectors on the disk (DWORD) */
      case GET_SECTOR_COUNT :

        HAL_MMC_GetCardInfo( hmmc, &halCardInfo );


        *(DWORD*)buf = halCardInfo.LogBlockNbr;
        res = RES_OK;
        break;

      /* Get R/W sector size (WORD) */
      case GET_SECTOR_SIZE :

         HAL_MMC_GetCardInfo( hmmc, &halCardInfo );

         *(WORD*)buf = halCardInfo.LogBlockSize;
         res = RES_OK;

         break;

      /* Get erase block size in unit of sector (DWORD) */
      case GET_BLOCK_SIZE :

         HAL_MMC_GetCardInfo( hmmc, &halCardInfo );

         *(DWORD*)buf = halCardInfo.LogBlockSize / MMC_DEFAULT_BLOCK_SIZE;
         res = RES_OK;

         break;

      default:
        res = RES_PARERR;
   }


   return res;
}

#endif  /* _USE_IOCTL == 1 */

