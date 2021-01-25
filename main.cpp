#include "pico_explorer.hpp"

using namespace pimoroni;

uint16_t buffer[PicoExplorer::WIDTH * PicoExplorer::HEIGHT];
PicoExplorer pico_explorer(buffer);

int main() {
    pico_explorer.init();

    // set the backlight to a value between 0 and 255
    // the backlight is driven via PWM and is gamma corrected by our
    // library to give a gorgeous linear brightness range.
    pico_explorer.set_backlight(100);

    pico_explorer.set_pen(255, 0, 0);

    while(true) {
        pico_explorer.pixel(Point(0, 0));
        // now we've done our drawing let's update the screen
        pico_explorer.update();
    }
}