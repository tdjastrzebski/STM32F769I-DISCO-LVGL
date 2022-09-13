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
#include "stm32f769i_discovery_ts.h"

#define SCREEN_WIDTH OTM8009A_800X480_WIDTH
#define SCREEN_HEIGHT OTM8009A_800X480_HEIGHT
#define DRAW_BUFFER_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10)
#define LCD_BPP 4  // bytes per pixel

extern UART_HandleTypeDef huart1;
extern DMA2D_HandleTypeDef hdma2d;
extern TIM_HandleTypeDef htim14;

LV_FONT_DECLARE(lv_font_montserrat_48)
static lv_disp_drv_t _disp_drv;                                     // lvgl display driver
ALIGN_32BYTES(static lv_color_t _lvDrawBuffer[DRAW_BUFFER_SIZE]);  // declare a buffer of 1/10 screen size

static void LogInfo(const char* format_msg, ...);
static void FlushBufferStart(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p);
static void FlushBufferComplete(DMA2D_HandleTypeDef* hdma2d);
static void LvglRefresh(TIM_HandleTypeDef* htim);
static void HelloWorld(void);
static void LvglInit(void);
static void TouchapadRead(lv_indev_drv_t* drv, lv_indev_data_t* data);

extern "C" void PreInit() {}

extern "C" void SysInit() {}

extern "C" void Init() {}

extern "C" void PostInit(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_TS_Init(800, 472);

	LvglInit();
	// HelloWorld();
	//lv_demo_widgets();
	lv_demo_benchmark();
	// lv_demo_stress();
	// lv_demo_music();

	LogInfo("Hello World!\n");

	// set interrupt handlers
	hdma2d.XferCpltCallback = FlushBufferComplete;
	htim14.PeriodElapsedCallback = LvglRefresh;
	// start LVGL timer 5ms
	HAL_TIM_Base_Start_IT(&htim14);  // Note: this interrupt must have "Preemption Priority" higher than DMA2D interrupt!
}

extern "C" void MainLoop(void) {
	static uint32_t pattern = 0xf0f0f0f0;
	static uint32_t shift = 0;
	HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, (pattern >> shift) & 0x1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	++shift %= 32;
	HAL_Delay(100);
}

static void HelloWorld(void) {
	// See: https://docs.lvgl.io/latest/en/html/widgets/label.html
	static lv_style_t fontStyle;
	lv_style_init(&fontStyle);
	lv_style_set_text_font(&fontStyle, &lv_font_montserrat_48);
	lv_style_set_text_color(&fontStyle, lv_color_black());
	lv_obj_t* screen = lv_scr_act();
	lv_obj_t* label = lv_label_create(screen);
	lv_obj_add_style(label, &fontStyle, 0);
	lv_label_set_text_static(label, "Hello World!");
	lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void LvglInit(void) {
	lv_init();
	// initize display
	static lv_disp_draw_buf_t draw_buf;                                                  // lvgl display draw buffer
	lv_disp_draw_buf_init(&draw_buf, _lvDrawBuffer, nullptr, DRAW_BUFFER_SIZE);  // Initialize the display buffer
	lv_disp_drv_init(&_disp_drv);                                                        // basic initialization
	_disp_drv.flush_cb = FlushBufferStart;                                               // set your driver function
	_disp_drv.draw_buf = &draw_buf;                                                      // assign the buffer to the display
	_disp_drv.hor_res = SCREEN_WIDTH;
	_disp_drv.ver_res = SCREEN_HEIGHT;
	lv_disp_drv_register(&_disp_drv);  // finally, register the driver
	// initialize input device
	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.read_cb = TouchapadRead;
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	lv_indev_drv_register(&indev_drv);
}

static void LvglRefresh(TIM_HandleTypeDef* htim) {
	lv_tick_inc(5);
	lv_task_handler();
}

static void FlushBufferStart(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* buffer) {
	uint32_t width = lv_area_get_width(area);
	uint32_t height = lv_area_get_height(area);
	uint32_t bufferLength = width * height * LCD_BPP;
	uint16_t x = area->x1;
	uint16_t y = area->y1;
	// copy buffer using DMA2D without Pixel Format Conversion (PFC) or Blending
	uint32_t destination = LCD_FB_START_ADDRESS + LCD_BPP * (y * SCREEN_WIDTH + x);
	SCB_CleanDCache_by_Addr((uint32_t*)buffer, bufferLength);  // flush d-cache to SRAM before starting DMA transfer
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.OutputOffset = (SCREEN_WIDTH - width);
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Instance = DMA2D;
	HAL_DMA2D_Init(&hdma2d);
	HAL_DMA2D_ConfigLayer(&hdma2d, 0);
	HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)buffer, destination, width, height);
}

static void FlushBufferComplete(DMA2D_HandleTypeDef* hdma2d) {
	lv_disp_flush_ready(&_disp_drv);
}

static void TouchapadRead(lv_indev_drv_t* drv, lv_indev_data_t* data) {
	static int16_t last_x = 0;
	static int16_t last_y = 0;
	static TS_StateTypeDef state;

	BSP_TS_GetState(&state);

	if (state.touchDetected != 0) {
		data->point.x = state.touchX[0];
		data->point.y = state.touchY[0];
		last_x = data->point.x;
		last_y = data->point.y;
		data->state = LV_INDEV_STATE_PR;
	} else {
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_REL;
	}
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

/* FlushBufferStart() without using DMA2D
static void FlushBufferStart(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* buffer) {
    int32_t srcLineSize = LCD_BPP * (area->x2 - area->x1 + 1);
    int32_t dstLineSize = LCD_BPP * SCREEN_WIDTH;
    char* src = (char*)buffer;
    char* dst = (char*)(LCD_FB_START_ADDRESS + LCD_BPP * (area->y1 * SCREEN_WIDTH + area->x1));

    for (int32_t y = area->y1; y < area->y2; y++) {
        // copy buffer to display line by line
        memcpy(dst, src, srcLineSize);
        src += srcLineSize;
        dst += dstLineSize;
    }
    memcpy(dst, src, srcLineSize);  // copy the last line
    SCB_CleanDCache_by_Addr((uint32_t*)LCD_FB_START_ADDRESS, SCREEN_WIDTH * SCREEN_HEIGHT * LCD_BPP);

    if (disp != NULL) lv_disp_flush_ready(disp);  // Indicate you are ready with the flushing
}
*/