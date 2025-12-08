#include "main_user.h"


#include "stm32h745i_discovery_qspi.h"
#include "stm32h745i_discovery_sdram.h"
#include "stm32h745i_discovery_bus.h"
#include "stm32h745i_discovery_errno.h"


/**
 * @brief initLtdcClocks initialize LTDC clocks
 */
void initLtdcClocks(void)
{
   // enable the LTDC and DMA2D clocks
   __HAL_RCC_LTDC_CLK_ENABLE();

   // enable GPIOs clock
   __HAL_RCC_GPIOI_CLK_ENABLE();
   __HAL_RCC_GPIOJ_CLK_ENABLE();
   __HAL_RCC_GPIOK_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();

   // LCD_DISP pin has to be manually controlled
   GPIO_InitTypeDef gpio_init_structure;
   gpio_init_structure.Pin = GPIO_PIN_7;
   gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
   HAL_GPIO_Init(GPIOD, &gpio_init_structure);

   // assert display enable LCD_DISP pin
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}


/**
 * @brief initBspQuadSpi initialize QSPI via BSP
 * @param error_handler callback to error handler
 */
void initBspQuadSpi(void (*error_handler)())
{
   BSP_QSPI_Init_t init;

   init.InterfaceMode = MT25TL01G_QPI_MODE;
   init.TransferRate = MT25TL01G_DTR_TRANSFER;
   init.DualFlashMode = MT25TL01G_DUALFLASH_ENABLE;

   if (BSP_QSPI_Init(0, &init) != BSP_ERROR_NONE)
   {
      error_handler();
   }

   if (BSP_QSPI_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
   {
      error_handler();
   }
}


/**
 * @brief initBspSdRam initialize SDRAM/FMC via BSP
 * @param error_handler callback to error handler
 */
void initBspSdRam(void (*error_handler)())
{
   BSP_SDRAM_DeInit(0);

   if (BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
   {
      error_handler();
   }
}
