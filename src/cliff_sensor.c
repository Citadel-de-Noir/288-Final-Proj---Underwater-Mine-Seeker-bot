
#include "./include/open_interface.h"
#include "./include/uart_interrupt.h"
#include "./include/lcd.h"
#include "./include/timer.h"
#include <stdio.h>

int main()
{
    timer_init();
    lcd_init();

    oi_t *sensor_data = oi_alloc();

    oi_init(sensor_data);
    //uart_interrupt_init();

    char buffer[100];

    while(1)
    {
        oi_update(sensor_data);
        /*
        sprintf(buffer,
                "Left Cliff: %d | Front Left: %d | Front Right: %d | Right Cliff: %d\r\n",
                sensor_data->cliffLeftSignal,
                sensor_data->cliffFrontLeftSignal,
                sensor_data->cliffFrontRightSignal,
                sensor_data->cliffRightSignal);

        uart_sendStr(buffer);
        */
        lcd_printf("Bump: %d Left: %4d\nFront Left: %4d\nFront Right: %4d\nRight: %4d \r\n", sensor_data->bumpLeft, sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal,
                sensor_data->cliffFrontRightSignal,
                sensor_data->cliffRightSignal);

        //lcd_printf("Bump Left: %d\nBump Right: %d", sensor_data->bumpLeft, sensor_data->bumpRight);
        if(sensor_data->bumpLeft == 1 || sensor_data->bumpRight == 1){
            oi_setWheels(0,0);
        }
        else{
            oi_setWheels(100,100);
        }
        timer_waitMillis(500);
    }
}

