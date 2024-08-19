/*
 * ws2812.h
 *
 *  Created on: Apr 26, 2024
 *      Author: lior
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"

#define WS2812_MSG_LENGTH 24	/* 24 bits per led*/
#define WS2812_24BIT_MASK 0xFFFFFF


typedef struct{
	uint8_t g;
	uint8_t r;
	uint8_t b;
}WS2812_RGBTypeDef;

typedef struct{
	double h;  // 0-360
	double s;  // 0-1
	double v;  // 0-1
}WS2812_HSVTypeDef;

typedef struct{
	size_t led_num;
	WS2812_RGBTypeDef grb;
}WS2812_LedRGBTypeDef;

typedef struct{
	size_t led_num;
	WS2812_HSVTypeDef hsv;
}WS2812_LedHSVTypeDef;




typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t tim_channel;
	uint32_t *packet_buf;
	size_t packet_buf_size;
	uint16_t *dma_buf;
	size_t dma_buf_size;
	uint32_t logical_one_ccr;
	uint32_t logical_zero_ccr;

}WS2812_HandleTypeDef;

HAL_StatusTypeDef ws2812_init(WS2812_HandleTypeDef *hpxl, TIM_HandleTypeDef *htim, uint32_t tim_channel, double timer_freq_hz,
		uint16_t dma_buf[], int dma_buf_size, uint32_t *packet_buf, size_t led_num);
void ws2812_set_color_rgb_v2(WS2812_HandleTypeDef *hpxl, WS2812_LedRGBTypeDef *led_rgb, size_t led_rgb_buf_size);
HAL_StatusTypeDef ws2812_write(WS2812_HandleTypeDef *hpxl);
HAL_StatusTypeDef ws2812_reset(WS2812_HandleTypeDef *hpxl);
HAL_StatusTypeDef ws2812_soft_reset(WS2812_HandleTypeDef *hpxl, double time);
HAL_StatusTypeDef ws2812_update_packet_buf(WS2812_HandleTypeDef *hpxl, WS2812_RGBTypeDef color);
void ws2812_set_color_rgb(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_RGBTypeDef color);

// not implemented functions
void ws2812_set_color_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_HSVTypeDef hsv);
void ws2812_set_color_hsv_v2(WS2812_HandleTypeDef *hpxl, WS2812_LedHSVTypeDef *led_hsv_buf, size_t led_hsv_buf_size);
bool ws2812_hsv2rgb(WS2812_HSVTypeDef hsv, WS2812_RGBTypeDef *grb);
bool ws2812_hsv2packet(const WS2812_HSVTypeDef hsv, uint32_t *pkt);
bool ws2812_rgb2packet(const WS2812_RGBTypeDef grb, uint32_t *pkt);
HAL_StatusTypeDef ws2812_write_v2(WS2812_HandleTypeDef *hpxl);

#endif /* INC_WS2812_H_ */
