/*
 * led_matrix.c
 *
 *  Created on: Feb 15, 2018
 *      Author: danielg
 */
#include "led_matrix.h"

int init_mat(led_mat_t* mat, uint8_t length, uint8_t height, uint8_t vert_horiz_, uint8_t num_of_bars, uint8_t strip_id)
{
	uint8_t bar_idx;
	if ((height > MAX_MATRIX_HEIGHT) || (length > MAX_MATRIX_LENGTH) || (num_of_bars > MAX_NUMBER_OF_BARS) || strip_id > MAX_ACTIVE_STRIPS)
	{
		return 1;
	}
	mat->mat_height = height;
	mat->mat_length = length;
	mat->strip_id = strip_id;
	mat->num_of_bars = num_of_bars;
	for (bar_idx=0; bar_idx<num_of_bars; bar_idx++)
	{
		mat->bars[bar_idx]->curr_val  = 0;
		mat->bars[bar_idx]->direction = vert_horiz_ ? BAR_DIR_UP : BAR_DIR_RIGHT;
		mat->bars[bar_idx]->rep_around_en = TRUE;
		mat->bars[bar_idx]->size = vert_horiz_ ? MAX_MATRIX_HIEGHT : MAX_MATRIX_LENGTH;
	}
	mat->strip_id = strip_id;
	mat->vert_horiz_ = vert_horiz_;
	return 0;
}

int reset_bar(led_mat_t* mat, uint8_t bar_id, uint8_t direction, uint8_t value, uint8_t size, uint8_t rep_around_en)
{
	if (( mat->vert_horiz_ && (direction == BAR_DIR_LEFT || direction = BAR_DIR_RIGHT)) ||
		(!mat->vert_horiz_ && (direction == BAR_DIR_UP   || direction = BAR_DIR_DOWN) ) ||
		(size > (mat->vert_horiz_ ? MAX_MATRIX_HIEGHT : MAX_MATRIX_LENGTH))             ||
		(value > size)                                                                  ||
		(bar_id > mat->num_of_bars))
	{
		return 1;
	}
	mat->bars[bar_id]->direction     = direction;
	mat->bars[bar_id]->curr_val      = value;
	mat->bars[bar_id]->size          = size;
	mat->bars[bar_id]->rep_around_en = rep_around_en;
	return 0;
}

int update_bar_value(led_mat_t* mat, uint8_t bar_id, uint8_t value)
{
	if ((value > mat->bars[bar_id]->size) || (bar_id > mat->num_of_bars))
	{
		return 1;
	}
	mat->bars[bar_id]->curr_val = value;
	return 0;
}

int inc_bar_by_val(led_mat_t* mat, uint8_t bar_id, uint8_t inc_val)
{
	uint16_t new_val;
	if (bar_id > mat->num_of_bars) return 1;
	new_val = mat->bars[bar_id]->curr_val + inc_val;
	if (new_val > mat->bars[bar_id]->size)
	{
		new_val = (mat->bars[bar_id]->rep_around_en) ? new_val%(mat->bars[bar_id]->size) : (mat->bars[bar_id]->size);
	}
	mat->bars[bar_id]->curr_val = new_val;
	return 0;
}

void display_led_matrix(led_mat_t* mat)
{
	uint8_t bar_id;
	g_pwr = strip_id==1 ? power_idx*2 : 0;
	r_pwr = strip_id==1 ? power_idx*2 : 0;
	b_pwr = strip_id==0 ? power_idx*2 : 0;

	for (bar_id=0; bar_id<MAX_LEDS_IN_STRIP; bar_id++)
	{
		LED_strips[strip_id][led_id][GREEN] = g_pwr;
		LED_strips[strip_id][led_id][RED]   = r_pwr;
		LED_strips[strip_id][led_id][BLUE]  = b_pwr;

	}
	update_GPIO_all_strips_mask(GPIO_PIN_5);
	update_driver_mask(GPIOB_PORT);
	drive_port_strips();
}
