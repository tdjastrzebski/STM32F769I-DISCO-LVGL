# STM32F769NI Discovery Kit (STM32F769I-DISCO) LCD Demo
This software package demonstrates how to set up LCD display on STM32F769I-DISCO board.
# VS Code Environment Setup
## Prerequisites
* NodeJS
* NPM
* Git
* Python and PIP
* VS Code
## pyocd install
* `pip install pyocd --upgrade` On Windows install as admin, otherwise install is local and PATH settings is required.
* `pyocd pack update`
* `pyocd pack find stm32f769`
* `pyocd pack install STM32F769NIHx`
## SVD file - MPU specific
Download from https://github.com/posborne/cmsis-svd/tree/master/data/STMicro and place in the Software root folder.
## Open OCD
* `npm install --global xpm@latest`
* `xpm install --global @xpack-dev-tools/openocd`
* Set `XDG_CACHE_HOME` env variable to `%USERPROFILE%\AppData\Local\Temp` to suppress symbol cache path error (Windows)
## GNU Arm Embedded Toolchain
* https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
* Set `MBED_GCC_ARM_PATH` env variable to `C:\Program Files (x86)\GNU Arm Embedded Toolchain\11 2021-q2\bin` (latest version Windows path).
* Add `MBED_GCC_ARM_PATH` env variable to Windows `Path` env variable (`%MBED_GCC_ARM_PATH%`) - if not set by the installer.
## Required NPM packages
* `npm install -g cppbuild`
* `npm install -g shx`
* `npm install -g @serialport/terminal`
## Required VS Code plug-ins
* `C/C++` (Microsoft)
* `Cortex-Debug` (marcus25) See: https://github.com/Marus/cortex-debug/wiki
## Recommended VS Code plug-ins
* `LinkerScript` (Zixuan Wang)
* `Arm Assembly` (dan-c-underwood)
* `Code Spell Checker` (Street Side Software)
* `Build++` (Tomasz Jastrzębski)
## Optional
* [ST-LINK Utility](https://github.com/stlink-org/stlink)
* [STSW-LINK007](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link007.html) ST boards firmware upgrade
* [STSW-LINK009](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html) ST-LINK probe V1/2 Windows USB driver

# References
## Discovery kit with STM32F769NI MCU docs
* https://www.st.com/en/evaluation-tools/32f769idiscovery.html
* https://www.st.com/en/evaluation-tools/32f769idiscovery.html#documentation
* https://www.st.com/en/evaluation-tools/32f769idiscovery.html#cad-resources

## STM32F769NI MPU docs
* https://www.st.com/en/microcontrollers-microprocessors/stm32f769ni.html
* https://www.st.com/en/microcontrollers-microprocessors/stm32f769ni.html#documentation
* [DS11532](https://www.st.com/resource/en/datasheet/stm32f769ni.pdf)
* [RM0410](https://www.st.com/resource/en/reference_manual/rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## How to add a BSP (Board Support Packages) to an STM32CubeIDE project?
https://st.force.com/community/s/article/how-to-add-a-bsp-to-an-stm32cubeide-project
