0.工程目录说明

STARTUP
启动目录
startup_stm32f10x_hd.s中包含了系统复位后最先运行的代码

CMSIS
内核目录
core_cm3.c是内核的驱动库，所有单片机的操作最终都会调用这里面的函数
system_stm32f10x.c用于系统时钟配置

FWLIB
固件库目录
使用库函数写代码时调用的函数都在这里面
其中misc是其它固件的固件库

USER
用户目录
需要自己写或者修改的代码就放在这里
stm32f10x_it.c	中断函数一半放在这里

DOC
文档目录
存放一些相关的文档
readme.txt 说明文档

1.程序运行流程
复位->SystemInit()(startup_stm32f10x_xx.s)->main()

2.时钟简介
(1)4个时钟源（HSI,HSE,LSI,LSE）加1个PLL时钟

HSI
由内部8MHz的RC振荡器产生，可直接作为系统时钟或在2分频后作为PLL输入。
HSI RC振荡器能够在不需要任何外部器件的条件下提供系统时钟。它的启动时间比HSE晶体振
荡器短。然而，即使在校准之后它的时钟频率精度仍较差。

HSE
高速外部时钟信号(HSE)由以下两种时钟源产生：
● HSE外部晶体/陶瓷谐振器
● HSE用户外部时钟

LSE
由一个32.768kHz的低速外部晶体或陶瓷谐振器产生。它为实时时钟或者其他定时功能提供
一个低功耗且精确的时钟源。

LSI
由内部40kHz的RC振荡器产生，担当一个低功耗时钟源的角色，它可以在停机和待机模式下保持运行，为独立看门狗和
自动唤醒单元提供时钟。LSI时钟频率大约40kHz(在30kHz和60kHz之间)。

PLL
主PLL以下述时钟源之一为输入，产生倍频的输出：
● HSI时钟除以2
● HSE或通过一个可配置分频器的PLL2时钟

(2)AHB,APB1,APB2,ADC,系统定时器分频器

AHB
将SYSCLK分频得到HCLK时钟
①、送给AHB总线、内核、内存和DMA使用的HCLK时钟。
②、通过分频后送给Cortex的系统定时器时钟。
③、直接送给Cortex的空闲运行时钟FCLK。
④、送给APB1分频器。
⑤、送给APB2分频器。

APB1
将HCLK时钟分频得到低速外设时钟PCLK1，
供电源接口、备份接口、CAN、USB、I2C1、I2C2、UART2、UART3、SPI2、窗口看门狗、Timer2、Timer3、Timer4等设备使用。

APB2
将HCLK时钟分频得到高速外设时钟PCLK2
供UART1、SPI1、Timer1、ADC1、ADC2、所有普通IO口(PA~PE)、第二功能IO口等设备使用。

ADC
将PCLK2时钟分频得到ADC采样时钟

系统定时器（滴答定时器）
将HCLK分频得到SysTick
