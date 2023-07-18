#ifndef GENERIC_PAD_PLUGIN_H_
#define GENERIC_PAD_PLUGIN_H_

#include <robotnik_pad/button.h>
#include <robotnik_pad/axes.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <unordered_map>

struct pad_element
{
  bool isAButton;
  int index;
  bool negativeAxes{false};
};

namespace pad_plugins
{
class GenericPadPlugin
{
public:
  typedef boost::shared_ptr<GenericPadPlugin> Ptr;

public:
  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns) = 0;
  virtual void execute(const std::vector<Button>& buttons, const std::vector<Axes>& axes) = 0;
  virtual ~GenericPadPlugin()
  {
  }

  //! Reads a parameter from the param server, and shows a message if parameter is not set
  template <typename T>
  bool readParam(const ros::NodeHandle& h, const std::string& name, T& value, const T& default_value,
                 bool required = false)
  {
    // parameter is read from node handle passed
    // required defines logger lever: if true, will show an error. if false, a warning
    if (h.hasParam(name) == false)
    {
      if (required == false)
      {
        ROS_WARN_STREAM("No parameter \"" << h.resolveName(name) << "\", using default value: " << default_value
                                          << ".");
      }
      else
      {
        ROS_ERROR_STREAM("No parameter \"" << h.resolveName(name) << "\", using default value: " << default_value
                                           << ".");
      }
      value = default_value;
      return false;
    }
    h.param<T>(name, value, default_value);
    return true;
  }

void stringParser()
{
  bool required = true;
  for(auto &it : button_axes_params_to_load_)
  {
    readParam(pnh_, it.first, it.second, it.second, required);
    auto found_plus = it.second.find("+");
    auto found_minus = it.second.find("-");
    pad_element element_to_fill;
    element_to_fill.isAButton = (found_plus==std::string::npos) && (found_minus==std::string::npos);
    
    if(!it.second.empty())
    {
      element_to_fill.index = std::stoi(it.second);
    }
    if(element_to_fill.index < 0 && !element_to_fill.isAButton)
    {
      element_to_fill.index = abs(element_to_fill.index);
      element_to_fill.negativeAxes = true;
    }    
    pad_elements_vector_.push_back(element_to_fill);
  }
  
}

void addNewParamToRead(std::string name)
{
  button_axes_params_to_load_.insert({name, ""});
}

float checkIfPadElementIsPressed(int index_to_check, const std::vector<Button> &buttons, const std::vector<Axes> &axes)
{
  float return_value{false};
  if(index_to_check > pad_elements_vector_.size())
  {
    ROS_ERROR("Index cannot be found %d", index_to_check);
    return -1;
  }
  auto element = pad_elements_vector_[index_to_check];
  if(element.isAButton)
  {
     return_value = buttons[element.index].isPressed();
  }else
  {
    return_value = element.negativeAxes ? axes[pad_elements_vector_[index_to_check].index].isPressedNeg() : axes[pad_elements_vector_[index_to_check].index].isPressedPos();
  }
  return return_value;
}

float checkIfPadElementIsReleased(int index_to_check, const std::vector<Button> &buttons, const std::vector<Axes> &axes)
{
  float return_value;
  if(index_to_check > pad_elements_vector_.size())
  {
    ROS_ERROR("Index cannot be found %d", index_to_check);
    return -1;
  }
  auto element = pad_elements_vector_[index_to_check];
  if(element.isAButton)
  {
     return_value = buttons[element.index].isReleased();
  }else
  {
    return_value = element.negativeAxes ? axes[pad_elements_vector_[index_to_check].index].isReleasedNeg() : axes[pad_elements_vector_[index_to_check].index].isReleasedPos();
  }
  return return_value;
}

protected:
  GenericPadPlugin()
  {
  }

protected:
  std::vector<Button> buttons_;
  std::vector<Axes> axes_;
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  std::unordered_map<std::string, std::string> button_axes_params_to_load_;
  std::vector<pad_element> pad_elements_vector_;
};
}
#endif  // GENERIC_PAD_PLUGIN_H_
