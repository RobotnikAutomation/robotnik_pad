#ifndef _AXES_
#define _AXES_


#include <iostream>
//! Class to save the state of the axes
class Axes
{
    bool is_released_pos_{false};
    bool is_released_neg_{false};
    float axis_value_{0.0};

public:
    Axes(){}

    void press(float value)
    {
        is_released_pos_ = (axis_value_ > 0 && !(value > 0));
        is_released_neg_ = (axis_value_ < 0 && !(value < 0));
		
        axis_value_ = value;
    }

    int isPressedPos(const double threshold = 0) const
    {

        return axis_value_ > abs(threshold);
    }

    int isPressedNeg(const double threshold = 0) const
    {
        return axis_value_ < -abs(threshold);
    }

    bool isReleasedPos() const
    {
        return is_released_pos_;
    }

    bool isReleasedNeg() const
    {
        return is_released_neg_;
    }

    void resetReleased()
    {
        if (is_released_pos_)
            is_released_pos_ = false;
        if (is_released_neg_)
            is_released_neg_ = false;
    }

    float getValue() const
    {
        return axis_value_;
    }
};

#endif  // _AXES_