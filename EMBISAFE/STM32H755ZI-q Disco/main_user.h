#ifndef __MAIN_USER_H
#define __MAIN_USER_H

#ifdef __cplusplus
extern "C" {
#endif


//! initialize LTDC clocks
void initLtdcClocks(void);

//! initialize QSPI via BSP
void initBspQuadSpi(void (*error_handler)());

//! initialize SDRAM/FMC via BSP
void initBspSdRam(void (*error_handler)());


#ifdef __cplusplus
}
#endif

#endif
