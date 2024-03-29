# STM32F769NI Discovery Kit (STM32F769I-DISCO) LCD Demo with LVGL and DMA2D aka Chrom-ART support
Content of this repo demonstrates how to use LCD display on [STM32F769I-DISCO](https://www.st.com/en/evaluation-tools/32f769idiscovery.html) board with [LVGL](https://github.com/lvgl/lvgl) embedded graphics library without OS (bare-metal). Example shows how to set up VS Code development environment, including build and OCD step debugging.  

This repo uses submodules, to clone execute:  
`git clone --recursive https://github.com/tdjastrzebski/STM32F769I-DISCO-LVGL`

![hello world](HelloWorld.JPG)
# VS Code Environment Setup
## Prerequisites
* NodeJS + NPM
* Python + PIP
* Git
* VS Code
## pyOCD
* `pip install pyocd --upgrade` On Windows install as admin, otherwise install is local and PATH setting is required.
* `pyocd pack update`
* `pyocd pack find stm32f769`
* `pyocd pack install STM32F769NIHx`
## OpenOCD
* Download the latest version from https://github.com/xpack-dev-tools/openocd-xpack/releases
or build it yourself following [this guide](https://github.com/Marus/cortex-debug/wiki/How-to-build-current-OpenOCD-version-on-Windows)
* Unzip it to `OpenOCD` folder in `C:\Program Files`
* Set `OpenOCD` system variable to 'C:\Program Files\OpenOCD'
* Add `%OpenOCD%\bin` to system PATH variable
* Set `XDG_CACHE_HOME` env variable to `%USERPROFILE%\AppData\Local\Temp` to prevent symbol cache path error
> Note: xpm utility does not really support global installations (yet), hence I suggest the above approach.
## SVD file - MPU specific
Download from https://github.com/posborne/cmsis-svd/tree/master/data/STMicro and place in the root folder.
## GNU Arm Embedded Toolchain
* Required version: **arm-none-eabi** (bare-metal target) [10.3-2021.07](https://developer.arm.com/downloads/-/gnu-a)
* Do NOT use latest versions 11.x and 12.x. Newer versions currently have known bugs which may impact build and/or debug process.
* Set `MBED_GCC_ARM_PATH` env variable to `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\10.3 2021.07\bin`
* Add `MBED_GCC_ARM_PATH` env variable to Windows `Path` env variable (`%MBED_GCC_ARM_PATH%`) - if not set by the installer.
## Required NPM packages
* `npm install -g cppbuild`
* `npm install -g shx`
* `npm install -g @serialport/terminal`
## Required VS Code plug-ins
* `C/C++` (Microsoft)
* `Cortex-Debug` (marcus25) See: https://github.com/Marus/cortex-debug/wiki
## VS Code config
* Set the default VS Code terminal to `Git Bash`. Otherwise, VS Code may try to execute task NPM packages as (e.g.) PowerShell scripts.
## Recommended VS Code plug-ins
* `LinkerScript` (Zixuan Wang)
* `Arm Assembly` (dan-c-underwood)
* `Code Spell Checker` (Street Side Software)
* `Build++` (Tomasz Jastrzębski)
## Optional
* [ST-LINK Utility](https://github.com/stlink-org/stlink)
* [STSW-LINK009](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html) ST-LINK USB Driver
* [STSW-LINK007](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link007.html) ST-LINK boards firmware upgrade

# References
## Discovery kit with STM32F769NI MCU docs
* [Discovery kit with STM32F769NI MCU](https://www.st.com/en/evaluation-tools/32f769idiscovery.html)
* [Documentation](https://www.st.com/en/evaluation-tools/32f769idiscovery.html#documentation)
* [Resources](https://www.st.com/en/evaluation-tools/32f769idiscovery.html#cad-resources)

## STM32F769NI MPU docs
* [STM32F769NI MPU](https://www.st.com/en/microcontrollers-microprocessors/stm32f769ni.html)
* [DS11532 Arm® Cortex®-M7 32b MCU+FPU, 462DMIPS, up to 2MB Flash/ 512+16+4KB RAM, USB OTG HS/FS, 28 com IF, LCD, DSI datasheet](https://www.st.com/resource/en/datasheet/stm32f769ni.pdf)
* [RM0410 STM32F76xxx and STM32F77xxx advanced Arm®-based 32-bit MCUs reference manual](https://www.st.com/resource/en/reference_manual/rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
* [PM0253 STM32F7 Series and STM32H7 Series Cortex®-M7 processor programming manual](https://www.st.com/resource/en/programming_manual/pm0253-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf)
## How to add a BSP (Board Support Packages) to an STM32CubeIDE project?
https://st.force.com/community/s/article/how-to-add-a-bsp-to-an-stm32cubeide-project
