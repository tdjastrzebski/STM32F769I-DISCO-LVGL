/*---------------------------------------------------------------------------------------------------
 *  Copyright (c) 2021-2023 Tomasz Jastrzębski. All rights reserved.
 *  Licensed under the GPL 3.0 License. See LICENSE file in the project root for license information.
 *-------------------------------------------------------------------------------------------------*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// to be included in the main.c "USER CODE 1" section
void PreInit(void);
// to be included in the main.c "USER CODE Init" section
void Init(void);
// to be included in the main.c "USER CODE SysInit" section
void SysInit(void);
// to be included in the main.c "USER CODE 2" section
void PostInit(void);
// to be included in the main.c "USER CODE 3" section
void MainLoop(void);

#ifdef __cplusplus
}
#endif