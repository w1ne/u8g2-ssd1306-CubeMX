# u8g2-ssd1306-CubeMX

u8g2 stm32 CubeMX HAL i2c ssd1306 noname display example. Contains implementation of u8g2 callback functions for the GM009605 display: i2c, 4 pins, no reset. Display adress 0x3C.

Code was tested on the Nucleo stm32F103 board, but should work on any ST MCU with i2c and CubeMX HAL support. CubeMX project for Atollic studio is included. u8g2 library should be added separately.

![GM009605 display i2c](https://raw.githubusercontent.com/w1ne/u8g2-ssd1306-CubeMX/master/img/GM009605.jpg "GM009605")

![GM009605 display i2c working](https://raw.githubusercontent.com/w1ne/u8g2-ssd1306-CubeMX/master/img/GM009605_u8g2_logo.jpg "GM009605 working")

For more information see https://github.com/olikraus/u8g2.
