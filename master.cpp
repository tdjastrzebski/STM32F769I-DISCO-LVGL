/*---------------------------------------------------------------------------------------------
 *  Copyright (c) 2022 Tomasz Jastrzębski. All rights reserved.
 *  Licensed under the MIT License. See License.md in the project root for license information.
 *--------------------------------------------------------------------------------------------*/
#include "master.h"

#include <cstdarg>
#include <cstdio>

#include "lvgl.h"
#include "lvgl/demos/lv_demos.h"
#include "main.h"
#include "stm32f769i_discovery_lcd.h"

#define SCREEN_WIDTH OTM8009A_800X480_WIDTH
#define SCREEN_HEIGHT OTM8009A_800X480_HEIGHT
#define DRAW_BUFFER_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10)

extern UART_HandleTypeDef huart1;
LV_FONT_DECLARE(lv_font_montserrat_48)
static void LogInfo(const char* format_msg, ...);
static void FlushBuffer(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p);
static void HelloWorld(void);
static void LvglInit(void);
static void DrawRectangle(lv_disp_drv_t* disp, uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2, uint32_t bgColor, uint32_t borderColor, uint32_t borderWidth);

extern "C" void PreInit() {
}

extern "C" void SysInit() {
}

extern "C" void Init() {
}

static lv_color_t _lvDrawBuffer[DRAW_BUFFER_SIZE];  // declare a buffer of 1/10 screen size

extern "C" void PostInit(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	// LvglInit();
	// lv_demo_widgets();
	// HelloWorld();
	LogInfo("Hello World!\n");
	//DrawRectangle(nullptr, 10, 20, 10, 20, 0x00ffffff, 0x00ff00ff, 2);
	uint32_t area[100] = {0};
	memset(area, 0x00ff00ff, 100*4);
	LL_FillBuffer(LTDC_ACTIVE_LAYER_BACKGROUND, (uint32_t*)LCD_FB_START_ADDRESS, 10, 10, 0, (uint32_t)area);
}

static void LvglInit(void) {
	lv_init();
	// static uint32_t* _ltdcFrameBuffer = (uint32_t*)LCD_FB_START_ADDRESS;
	static lv_disp_draw_buf_t _draw_buf;                                       // lvgl display draw buffer
	lv_disp_draw_buf_init(&_draw_buf, _lvDrawBuffer, NULL, DRAW_BUFFER_SIZE);  // Initialize the display buffer
	static lv_disp_t* _disp;                                                   // lvgl display
	static lv_disp_drv_t _disp_drv;                                            // lvgl display driver

	lv_disp_drv_init(&_disp_drv);      // basic initialization
	_disp_drv.flush_cb = FlushBuffer;  // set your driver function
	_disp_drv.draw_buf = &_draw_buf;   // assign the buffer to the display
	//_disp_drv.rounder_cb = my_lv_rounder_cb;  // set rounder function
	_disp_drv.hor_res = SCREEN_WIDTH;
	_disp_drv.ver_res = SCREEN_HEIGHT;
	//_disp_drv.user_data = hdsi;
	_disp = lv_disp_drv_register(&_disp_drv);  // finally register the driver
}

static void HelloWorld(void) {
	lv_style_t fontStyle;
	lv_style_init(&fontStyle);
	lv_style_set_text_font(&fontStyle, &lv_font_montserrat_48);
	lv_style_set_text_color(&fontStyle, lv_color_white());
	lv_obj_t* screen = lv_scr_act();
	lv_obj_t* label = lv_label_create(screen);
	lv_label_set_text(label, "Hello World!");
	lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 200);
}

static void FlushBuffer(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p) {
	uint16_t x = area->x1;
	uint16_t y = area->y1;
	uint16_t w = lv_area_get_width(area);
	uint16_t h = lv_area_get_height(area);
	uint32_t source = (uint32_t)color_p;
	uint32_t destination = LCD_FB_START_ADDRESS + 4 * (y * SCREEN_WIDTH + x);
	uint16_t offset = SCREEN_WIDTH - w;
	LL_FillBuffer(LTDC_ACTIVE_LAYER_BACKGROUND, (uint32_t*)destination, w, h, offset, source);
	if (drv != nullptr) lv_disp_flush_ready(drv);
}

extern "C" void MainLoop(void) {
	static uint32_t pattern = 0xffff0000;
	static uint32_t shift = 0;
	HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, (pattern >> shift) & 0x1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	++shift %= 32;
	HAL_Delay(10);
	// lv_tick_inc(10);
	// lv_task_handler();
}

void LogInfo(const char* format_msg, ...) {
	static char textBuffer[64];
	va_list args;
	va_start(args, format_msg);
	int len = vsnprintf(textBuffer, sizeof(textBuffer), format_msg, args);
	va_end(args);

	if (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t*)textBuffer, len, 0xFFFFFFFF)) {
		Error_Handler();
	}
}

static void DrawRectangle(lv_disp_drv_t* disp, uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2, uint32_t bgColor, uint32_t borderColor, uint32_t borderWidth) {
	lv_area_t area = {};
	area.x1 = x1;
	area.x2 = x2;
	area.y1 = y1;
	area.y2 = y2;
	uint32_t width = x2 - x1 + 1;
	uint32_t height = y2 - y1 + 1;
	uint32_t size = width * height;
	// lv_color_t* content = new lv_color_t[size];
	lv_color_t content[size];
	lv_color_t c;
	c.full = bgColor;
	for (uint32_t i = borderWidth * width; i < width * (height - borderWidth); i++) content[i] = c;  // fill in
	c.full = borderColor;
	for (uint32_t i = 0; i < borderWidth * width; i++) content[i] = c;                // bottom border
	for (uint32_t i = (height - borderWidth) * width; i < size; i++) content[i] = c;  // top border
	for (uint32_t i = 0; i < borderWidth; i++) {
		for (uint32_t j = i + (borderWidth * width); j < size - (borderWidth * width); j += width) content[j] = c;              // left border
		for (uint32_t j = width - i - 1 + (borderWidth * width); j < size - (borderWidth * width); j += width) content[j] = c;  // right border
	}

	FlushBuffer(nullptr, &area, content);
	// delete[] content;
	// std::free(content)
}