# led_matrix_panel
A library to control led matrix panels from China

You can use this for different MCU, the current implement is for STM32 user STM32CUBE HAL

All the functions hardware dependent is in led_matrix_porting.c, so you have to modify this file to suit your MCU

The coordinate of panel: ORIGIN: bottom right corner
                
15 14 13 12 11 10 9 8  
7  6  5  4  3  2  1 0 <- ORIGIN
TODO: FIX WRONG SCANNING ORDER
