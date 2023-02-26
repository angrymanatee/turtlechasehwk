/**
 * Utility functions for turtle chase homework.
 * 
 * Normally I hate these kinda things because they become unruly pretty quickly,
 * but this is a pretty small homework, so whatever.
*/


#include "turtlechasehwk/utils.hpp"

int turtlechasehwk::freqToPeriodMs(double freq_hz)
{
    return static_cast<int>(1000.0 / freq_hz);
}
