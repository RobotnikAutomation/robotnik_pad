#ifndef _AXES_
#define _AXES_


#include <iostream>
//! Class to save the state of the buttons
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

  int isPressedPos() const
  {
    return is_pressed_pos;
  }
	int isPressedNeg() const
  {
    return is_pressed_neg;
  }

  bool isReleasedPos() const
  {
    return is_released_pos;
  }

	bool isReleasedNeg() const
  {
    return is_released_neg;
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