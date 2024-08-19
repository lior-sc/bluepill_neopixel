/*
 * WS2812.c
 *
 *  Created on: Apr 26, 2024
 *      Author: lior
 */

#include "ws2812.h"
#include "stm32f1xx_hal.h"
#include "math.h"

#define LOGICAL_ONE_HIGH_PULSE_WIDTH_NS 	800
#define LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS 	400
#define MSG_BIT_TIME_ELAPSED_NS				1250
#define MSG_MAX_TIMING_ERROR_NS				150

HAL_StatusTypeDef ws2812_init(WS2812_HandleTypeDef *hpxl, TIM_HandleTypeDef *htim, uint32_t tim_channel, double timer_freq_hz,
		uint16_t dma_buf[], int dma_buf_size, uint32_t *packet_buf, size_t led_num){

	double _one_ccr_value = 0;
	double _zero_ccr_value = 0;
	double _one_timing_error = 0;
	double _zero_timing_error = 0;
	double _msg_timing_error = 0;
	double counter_tick_period_ns = (1 / timer_freq_hz) * 1e9;
	double timer_ARR = (double)htim->Instance->ARR;
	/**
	 * calculate CCR values for logical 1 and logical 0 on ws2812 communication protocol
	 */
	_zero_ccr_value = round(LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS / counter_tick_period_ns);
	_one_ccr_value =  round(LOGICAL_ONE_HIGH_PULSE_WIDTH_NS / counter_tick_period_ns);

	/**
	 * make sure timing tolerances stand up to WS2812 requirements
	 */
	_zero_timing_error = fabs(LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS - (_zero_ccr_value * counter_tick_period_ns));
	_one_timing_error =  fabs(LOGICAL_ONE_HIGH_PULSE_WIDTH_NS - (_one_ccr_value * counter_tick_period_ns));
	_msg_timing_error =  fabs(MSG_BIT_TIME_ELAPSED_NS - timer_ARR * counter_tick_period_ns);

	if(_zero_timing_error > MSG_MAX_TIMING_ERROR_NS ||
			_one_timing_error > MSG_MAX_TIMING_ERROR_NS ||
			_msg_timing_error > MSG_MAX_TIMING_ERROR_NS){
		// timing error is too big. go back and redefine timer to stand up to tolerance requirements
		return false;
	}

	/**
	 * set metadata for led strip
	 */
	hpxl->htim = htim;
	hpxl->tim_channel = tim_channel;
	hpxl->packet_buf_size = led_num;
	hpxl->packet_buf = packet_buf;
	hpxl->dma_buf = dma_buf;
	hpxl->dma_buf_size = dma_buf_size;
	hpxl->logical_zero_ccr = _zero_ccr_value;
	hpxl->logical_one_ccr = _one_ccr_value;

	// if we got here it means all went well
	return true;
}

/**
 * @brief This function updates the DMA buffer with the required led colors. the functions goes over the entire
 * 		  LED array and changes the colors of the required LEDS
 *
 * @param led_buf - this is and int array that contains the leds we want to set the color to
 * @param led_buf_size - this is the size of the led_buf
 * @param dma_buf[] - this is a pointer to the dma_buffer
 * @param dma_buf_size - this is the size of the DMA buffer
 */
void ws2812_set_color_rgb(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_RGBTypeDef color){
	uint32_t packet = 0;
	uint32_t mask = 0;
	uint32_t state = 0;
	int dma_index = 0;
	int led_index = 0;

	// create WS2812 message packet
	packet |= color.g << 16;
	packet |= color.r << 8;
	packet |= color.b;

	// convert message packet to DMA CCR values
	for(int i=0; i < led_buf_size; i++){
		for(int j=0; j < 24; j++)
		{
			// determine bit state
			mask = 1 << (23 - j);
			state = (packet & mask) >> (23 - j);
			led_index = led_buf[i];
			dma_index = (led_index * 24) + j +1;


			// set PWM CCR value to DMA buffer
			if(state == 0){
				hpxl->dma_buf[dma_index] = hpxl->logical_zero_ccr;
				hpxl->dma_buf[dma_index] = hpxl->logical_zero_ccr;
			}
			else{
				hpxl->dma_buf[dma_index] = hpxl->logical_one_ccr;
			}
		}
	}
	return;
}

HAL_StatusTypeDef ws2812_write(WS2812_HandleTypeDef *hpxl){
	HAL_StatusTypeDef ret = HAL_ERROR;

	// send data via PWM_DMA functionality
	hpxl->dma_buf[0] = 0;
	hpxl->dma_buf[hpxl->dma_buf_size-1] = 0;	// set 0 PWM duty cycle to reset transmission
	ret = HAL_TIM_PWM_Start_DMA(hpxl->htim, hpxl->tim_channel, (uint32_t *)hpxl->dma_buf, hpxl->dma_buf_size);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop_DMA(hpxl->htim, hpxl->tim_channel);

	return ret;
}

HAL_StatusTypeDef ws2812_reset(WS2812_HandleTypeDef *hpxl){
	HAL_StatusTypeDef ret = HAL_ERROR;

	// send logical zeros to DMA. this will set an RGB value of 0 to all leds
	for(int i=0; i < hpxl->dma_buf_size-1; i++){
		hpxl->dma_buf[i] = hpxl->logical_zero_ccr;
	}

	// send dma buffer data to LEDS
	ret = ws2812_write(hpxl);

	return ret;

}

void ws2812_set_color_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_HSVTypeDef hsv){
	WS2812_RGBTypeDef rgb = {0,0,0};
	double R = 0;
	double G = 0;
	double B = 0;

	double h_prime = hsv.h / 60;
	double c = hsv.v * hsv.s;
	double x =  c * (1 - fabs(fmod(h_prime, 2) - 1));
	double m = hsv.v - c;

	if(hsv.h > 360.0 || hsv.h < 0.0 || hsv.s < 0.0 || hsv.s > 1.0 || hsv.v < 0.0 || hsv.v > 1.0){
		return;
	}

	if(h_prime >= 0 && h_prime < 1){
		R = c;
		G = x;
		B = 0;
	}
	else if(h_prime >= 1 && h_prime < 2){
		R = x;
		G = c;
		B = 0;
	}
	else if(h_prime >= 2 && h_prime < 3){
		R = 0;
		G = c;
		B = x;
	}
	else if(h_prime >= 3 && h_prime < 4){
		R = 0;
		G = x;
		B = c;
	}
	else if(h_prime >= 4 && h_prime < 5){
		R = x;
		G = 0;
		B = c;
	}
	else if(h_prime >= 5 && h_prime < 6){
		R = c;
		G = 0;
		B = x;
	}

	rgb.r = (uint8_t)(255 * (R+m));
	rgb.g = (uint8_t)(255 * (G+m));
	rgb.b = (uint8_t)(255 * (B+m));

	ws2812_set_color_rgb(hpxl, led_buf, led_buf_size, rgb);

	return;
}

HAL_StatusTypeDef ws2812_soft_reset(WS2812_HandleTypeDef *hpxl, double time){
	HAL_StatusTypeDef ret = HAL_ERROR;

	// send logical zeros to DMA. this will set an RGB value of 0 to all leds
	for(int j=0; j<1000;j++){
		for(int i=1; i <= hpxl->dma_buf_size-1; i++){
			hpxl->dma_buf[i] = hpxl->dma_buf[i+1];
		}
		ret = ws2812_write(hpxl);
		HAL_Delay((uint32_t)(time/1000));

	}


	// send dma buffer data to LEDS
	ret = ws2812_write(hpxl);

	return ret;
}

void ws2812_set_color_rgb_v2(WS2812_HandleTypeDef *hpxl, WS2812_LedRGBTypeDef *led_rgb_buf, size_t led_rgb_buf_size){
	for(size_t i=0; i<led_rgb_buf_size; i++){
		ws2812_rgb2packet(led_rgb_buf[i].grb, &hpxl->packet_buf[led_rgb_buf[i].led_num]);
	}
}

void ws2812_set_color_hsv_v2(WS2812_HandleTypeDef *hpxl, WS2812_LedHSVTypeDef *led_hsv_buf, size_t led_hsv_buf_size){

	for(size_t i=0; i< led_hsv_buf_size; i++){
		ws2812_hsv2packet(led_hsv_buf[i].hsv, &hpxl->packet_buf[led_hsv_buf[i].led_num]);
	}
}

bool ws2812_hsv2rgb(WS2812_HSVTypeDef hsv, WS2812_RGBTypeDef *grb){
	double r_tag = 0;
	double g_tag = 0;
	double b_tag = 0;

	double h_prime = hsv.h / 60.0;
	double c = hsv.v * hsv.s;
	double x =  c * (1 - fabs(fmod(h_prime, 2) - 1));
	double m = hsv.v - c;

	if(hsv.h > 360.0 || hsv.h < 0.0 || hsv.s < 0.0 || hsv.s > 1.0 || hsv.v < 0.0 || hsv.v > 1.0){
		return false;
	}

	if(h_prime >= 0 && h_prime < 1){
		r_tag = c;
		g_tag = x;
		b_tag = 0;
	}
	else if(h_prime >= 1 && h_prime < 2){
		r_tag = x;
		g_tag = c;
		b_tag = 0;
	}
	else if(h_prime >= 2 && h_prime < 3){
		r_tag = 0;
		g_tag = c;
		b_tag = x;
	}
	else if(h_prime >= 3 && h_prime < 4){
		r_tag = 0;
		g_tag = x;
		b_tag = c;
	}
	else if(h_prime >= 4 && h_prime < 5){
		r_tag = x;
		g_tag = 0;
		b_tag = c;
	}
	else if(h_prime >= 5 && h_prime < 6){
		r_tag = c;
		g_tag = 0;
		b_tag = x;
	}

	grb->g = (uint8_t)(255.0 * (g_tag+m));
	grb->r = (uint8_t)(255.0 * (r_tag+m));
	grb->b = (uint8_t)(255.0 * (b_tag+m));

	return true;
}

bool ws2812_hsv2packet(const WS2812_HSVTypeDef hsv, uint32_t *pkt){
//	define local variables
	WS2812_RGBTypeDef grb = {0,0,0};

//	convert hsv input to grb type
	if(!ws2812_hsv2rgb(hsv, &grb)){
		return false;
	}

//	create packet
	*pkt = ((grb.g << 16) | (grb.r << 8) | grb.b) & WS2812_24BIT_MASK;

	return true;
}

bool ws2812_rgb2packet(const WS2812_RGBTypeDef grb, uint32_t *pkt){
	*pkt = (uint32_t)(((grb.g << 16) | (grb.r << 8) | grb.b) & WS2812_24BIT_MASK);
	return true;
}

HAL_StatusTypeDef ws2812_write_v2(WS2812_HandleTypeDef *hpxl){
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint32_t state = 0;
	uint32_t mask = 0;
	size_t dma_index = 0;

	// update DMA buffer from packet buffer
	for(size_t i=0; i < hpxl->packet_buf_size; i++){
		for(int j=0; j < WS2812_MSG_LENGTH; j++)
		{
			// determine bit state
			mask = 1 << (23 - j);
			state = (hpxl->packet_buf[i] & mask) >> (23 - j);
			dma_index = (i * 24) + j + 1;

			// set PWM CCR value to DMA buffer
			if(state == 0){
				hpxl->dma_buf[dma_index] = hpxl->logical_zero_ccr;
			}
			else{
				hpxl->dma_buf[dma_index] = hpxl->logical_one_ccr;
			}
		}
	}

	// send data via PWM_DMA functionality
	hpxl->dma_buf[0] = 0;
	hpxl->dma_buf[hpxl->dma_buf_size-1] = 0;	// set 0 PWM duty cycle to reset transmission
	ret = HAL_TIM_PWM_Start_DMA(hpxl->htim, hpxl->tim_channel, (uint32_t *)hpxl->dma_buf, hpxl->dma_buf_size);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop_DMA(hpxl->htim, hpxl->tim_channel);

	return ret;
}
