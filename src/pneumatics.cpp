#include "globals.hpp"
#include "pneumatics.hpp"

bool switchState(bool state){
    return state==LOW ? HIGH : LOW;
}
