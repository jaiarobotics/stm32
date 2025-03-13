#ifndef __JAIA_MAIN_H
#define __JAIA_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx.h"
#include "stm32l433xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_adc.h"
#include "oem_library.h"
#include "stdint.h"
#include "string.h"

#include <../nanopb/pb_encode.h>
#include <../nanopb/pb_decode.h>

#include "../Messages/sensor_core.pb.h"
#include "../Messages/atlas_scientific__oem_ec.pb.h"
#include "../Messages/atlas_scientific__oem_do.pb.h"
#include "../Messages/atlas_scientific__oem_ph.pb.h"

/*struct boot_vectable_ {
	uint32_t Initial_SP;
	void (*Reset_Handler)(void);
};*/

//void jumpToBootloader(void);
//void I2C_Scan(void);
//void startAtlasChips(void);

#ifdef __cplusplus
}
#endif

#endif /* __JAIA_MAIN_H */
