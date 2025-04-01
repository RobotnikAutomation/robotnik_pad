#ifndef _AXES_
#define _AXES_


#include <iostream>
//! Class to save the state of the axes
class Axes
{
    int is_pressed_pos{0};
    int is_pressed_neg{0};
    bool is_released_pos{false};
    bool is_released_neg{false};
    float axis_value{0.0};

public:
    Axes(){}

    void press(float value)
    {
        bool isValuePositive = (value > 0);
        bool isValueNegative = (value < 0);

        is_released_pos = (is_pressed_pos && !isValuePositive);
        is_released_neg = (is_pressed_neg && !isValueNegative);

        is_pressed_pos = isValuePositive;
        is_pressed_neg = isValueNegative;
		
        axis_value = value;
    }

    int isPressedPos(const double threshold = 0) const
    {
        return is_pressed_pos && axis_value > abs(threshold);
    }

    int isPressedNeg(const double threshold = 0) const
    {
        return is_pressed_neg && axis_value < -abs(threshold);
    }

    bool isReleasedPos(const double threshold = 0) const
    {
        return is_released_pos && axis_value <= abs(threshold);
    }

    bool isReleasedNeg(const double threshold = 0) const
    {
        return is_released_neg && axis_value >= -abs(threshold);
    }

    void resetReleased()
    {
        if (is_released_pos)
            is_released_pos = false;
        if (is_released_neg)
            is_released_neg = false;
    }

    float getValue() const
    {
        return axis_value;
    }
};

#endif  // _AXES_