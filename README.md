# stm32f103
野火指南者相关代码的练习
为了方便以后下载，除了模板工程，其它工程只包含USER目录

## 开发工具版本信息
![开发工具版本信息](https://github.com/ysx0505/stm32f103/blob/master/000.PNG)

------
## stm32f103的时钟树
![stm32f103的时钟树](https://github.com/ysx0505/stm32f103/blob/master/001.PNG)

------
## 启动文件（.s）的选择

tip:芯片的容量类型可以在stm32选型手册或数据手册中查看

小容量型设备有STM32F101xx、STM32F102xx和STM32F103xx单片机,其flash大小在16到32k字节之间。
对应的启动文件名称为：startup_stm32f10x_ld.s

小容量超值型设备有STM32F100xx单片机,其flash大小在16到32k字节之间。
对应的启动文件名称为：startup_stm32f10x_ld.s

中容量型设备有STM32F101xx、STM32F102xx和STM32F103xx单片机,其flash大小在64到128k字节之间。
对应的启动文件名称为：startup_stm32f10x_md.s

中容量超值型设备有STM32F100xx单片机,其flash大小在64到128k字节之间。
对应的启动文件名称为：startup_stm32f10x_md_vl.s

大容量型设备有STM32F101xx和STM32F103xx单片机,其flash大小在256到512k字节之间。
对应的启动文件名称为：startup_stm32f10x_hd.s

大容量超值型设备有STM32F100xx单片机,其flash大小在256到512k字节之间。
对应的启动文件名称为：startup_stm32f10x_hd_vl.s

超大容量设备有STM32F101xx和STM32F103xx单片机,其flash大小在512到1024k字节之间。
对应的启动文件名称为：startup_stm32f10x_xl.s

互联网型设备有STM32F105xx和STM32F107xx单片机
对应的启动文件名称为：startup_stm32f10x_cl.s
