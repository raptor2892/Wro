#include <Arduino.h>

#include "line_follower.h"

void setup() {
    beginLineFollower();
}

void loop() {
    updateLineFollower();
    delay(5);
}

