#ifndef _BUTTON_
#define _BUTTON_

//! Class to save the state of the buttons
class Button
{
  int is_pressed_;
  bool is_released_;

public:
  Button()
  {
    is_pressed_ = 0;
    is_released_ = false;
  }

  //! Set the button as 'pressed'/'released'
  void Press(int value)
  {
    if (is_pressed_ and !value)
    {
      is_released_ = true;
    }
    else if (is_released_ and value)
      is_released_ = false;

    is_pressed_ = value;
  }

  int IsPressed()
  {
    return is_pressed_;
  }

  bool IsReleased()
  {
    bool b = is_released_;
    is_released_ = false;
    return b;
  }
};

#endif  // _BUTTON_