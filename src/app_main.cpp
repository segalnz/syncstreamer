#include <Arduino.h>

extern "C" void app_main(void)
{
    initArduino();
    setup();
    for (;;) {
        loop();
    }
}
