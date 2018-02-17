#ifndef _WS2812B_MULTI_STRIP_DRIVER_H
#define _WS2812B_MULTI_STRIP_DRIVER_H

#define MAX_SUPPORTED_NUM_OF_STRIPS (20)
#define MAX_SUPPORTED_LEDS_IN_STRIP (600)
#define NUM_OF_CFG_BYTES_PER_LED (3)
#define BITS_TO_CONFIGURE_ONE_LED (24)
#define MAX_LEDS_IN_STRIP (512)
#define MAX_ACTIVE_STRIPS (1)
#define BITS_IN_BYTE (8)
#define GPIOB_PORT 0
#define GPIOC_PORT 1
#define LED_MAX_POWER (150)

enum {
	GREEN 	= 0,
	RED 	= 1,
	BLUE 	= 2
};

#define GET_STRIP_PORT(strip) (((strip == 0) || (strip == 1) || (strip == 2)) ? GPIOB_PORT : GPIOC_PORT)
#define GET_STRIP_GPIO(strip) (strip == 0 ? 10 : strip == 1 ? 5 : strip == 2 ? 6 : 0)

void drive_port_strips(void);
void update_driver_mask(int port_idx);
void update_GPIO_all_strips_mask(uint16_t update_mask);
void wait_x_msec(int16_t msec_to_wait);

#endif  /* _WS2812B_MULTI_STRIP_DRIVER_H */
