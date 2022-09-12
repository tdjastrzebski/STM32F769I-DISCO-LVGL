/*---------------------------------------------------------------------------------------------
 *  Copyright (c) 2022 Tomasz Jastrzębski. All rights reserved.
 *  Licensed under the MIT License. See License.md in the project root for license information.
 *--------------------------------------------------------------------------------------------*/
#include "master.h"

#include <cstdarg>
#include <cstdio>

#include "main.h"
#include "stm32f769i_discovery_lcd.h"

extern UART_HandleTypeDef huart1;
void LogInfo(const char* format_msg, ...);

extern "C" void PreInit() {
}

extern "C" void SysInit() {
}

extern "C" void Init() {
}

extern "C" void PostInit(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, 240, (uint8_t*)"Hello World!", CENTER_MODE);
	LogInfo("Hello World!\n");
}

extern "C" void MainLoop(void) {
	static uint32_t pattern = 0xf0f0f0f0;
	static uint32_t shift = 0;
	HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, (pattern >> shift) & 0x1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	++shift %= 32;
	HAL_Delay(200);
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
