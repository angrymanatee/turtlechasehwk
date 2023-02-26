
#pragma once

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>


namespace turtlechasehwk {

int freqToPeriodMs(double freq_hz);

template <class T>
T clipToBounds(T input, T lower, T upper)
{
    if (input < lower) {
        return lower;
    } else if (input > upper) {
        return upper;
    } else {
        return input;
    }
}


}
