#include "pico_explorer.hpp"

using namespace pimoroni;

uint16_t buffer[PicoExplorer::WIDTH * PicoExplorer::HEIGHT];
PicoExplorer pico_explorer(buffer);

int main() {
    pico_explorer.init();

    pico_explorer.set_pen(255, 0, 0);

    while(true) {
        pico_explorer.pixel(Point(0, 0));
        // now we've done our drawing let's update the screen
        pico_explorer.update();
    }
}