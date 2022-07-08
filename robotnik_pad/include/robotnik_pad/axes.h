#ifndef _AXES_
#define _AXES_

//! Class to save the state of the buttons
class Axes
{
  int is_pressed_pos, is_pressed_neg;
  bool is_released_pos, is_released_neg;
	float axis_value;

public:
  Axes()
  {
    is_pressed_pos = 0;
		is_pressed_neg = 0;
    is_released_pos = false;
		is_released_neg = false;
		axis_value = 0.0;
  }

  //! Set the button as 'pressed'/'released'
  void press(float value)
  {
    if (is_pressed_pos and !(value > 0))
    {
      is_released_pos = true;
    }
    else if (is_released_pos and (value > 0))
			is_released_pos = false;

		if (is_pressed_neg and !(value < 0))
		{
			is_released_neg = true;
		}
		else if (is_released_neg and (value < 0))
			is_released_neg = false;

		if (value > 0)
			is_pressed_pos = true;
		else
			is_pressed_pos = false;

		if (value < 0)
			is_pressed_neg = true;
		else
			is_pressed_neg = false;

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
