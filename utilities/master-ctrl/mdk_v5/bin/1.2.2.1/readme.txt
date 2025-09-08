
智能托盘
srv1221，版本号：1.2.0.4->1.2.2.1

修改系统时钟频率从144M->104M，spi频率和can频率跟随改变
    涉及文件：FreeRTOSConfig.h，at32f415_clock.c，ax_can.c
修改灯带灯珠数量到56个
修复ota bug
添加底盘控制的通用指令，使用新版本编号