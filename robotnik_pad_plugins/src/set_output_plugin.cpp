#include <robotnik_pad_plugins/set_output_plugin.h>

namespace pad_plugins
{
PadPluginSetOutput::PadPluginSetOutput()
{
}

PadPluginSetOutput::~PadPluginSetOutput()
{
}

void PadPluginSetOutput::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  activate_ = false;
  deactivate_ = false;
  output_sent_ = false;
  bool required = true;
  bool not_required = false;

  pnh_ = ros::NodeHandle(nh, plugin_ns);

  readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/axis_set_output", axis_set_output_, axis_set_output_, required);
  readParam(pnh_, "base_hw_set_output_name", base_hw_set_output_name_, base_hw_set_output_name_, required);
  readParam(pnh_, "modbus_set_output_name", modbus_set_output_name_, modbus_set_output_name_, required);
  readParam(pnh_, "use_plc", use_plc_, false, not_required);
  readParam(pnh_, "has_feedback", has_feedback_, false, not_required);
  pnh_.getParam("outputs", outputs_);
  pnh_.getParam("inputs", inputs_);
  pnh_.getParam("output_values", output_values_);
  pnh_.getParam("input_values", input_values_);
  // readParam(pnh_, "outputs", outputs_, void_int_list, required);
  // readParam(pnh_, "output_values", output_values_, void_bool_list, not_required);
  // readParam(pnh_, "inputs", inputs_, void_int_list, not_required);
  // readParam(pnh_, "input_values", input_values_, void_bool_list, not_required);
  timeout_ = 1;
  readParam(pnh_, "timeout", timeout_, timeout_, not_required);

  base_hw_io_topic_name_ = "";
  readParam(pnh_, "base_hw_io_topic_name", base_hw_io_topic_name_, base_hw_io_topic_name_, not_required);
  modbus_io_topic_name_ = "";
  readParam(pnh_, "modbus_io_topic_name", modbus_io_topic_name_, modbus_io_topic_name_, not_required);
  int n = outputs_.size();
  int n_val = output_values_.size();
  if (n != n_val)
  {
    output_values_.clear();
    for (int i = 0; i < n; i++)
      output_values_[i] = true;
  }
  if (has_feedback_)
  {
    n = inputs_.size();
    if (n == 0)
      has_feedback_ = false;
    else if (base_hw_io_topic_name_ == "" && !use_plc_)
      has_feedback_ = false;
    else if (modbus_io_topic_name_ == "" && use_plc_)
      has_feedback_ = false;
    else
    {
      n_val = input_values_.size();
      if (n != n_val)
      {
        input_values_.clear();
        for (int j = 0; j < n; j++)
          input_values_[j] = true;
      }
    }
  }

  // Service client
  std::string service_name = use_plc_ ? modbus_set_output_name_ : base_hw_set_output_name_;
  set_output_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(service_name);

  //Subscriber
  if (has_feedback_)
  {
    std::string topic_name = use_plc_ ? modbus_io_topic_name_ : base_hw_io_topic_name_;
    io_sub_ = nh_.subscribe<robotnik_msgs::inputs_outputs>(topic_name, 1, &PadPluginSetOutput::ioCb, this);
  }
}

void PadPluginSetOutput::ioCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg)
{
  io_ = *msg;
}

void PadPluginSetOutput::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (axes[axis_set_output_] > 0.95)
    {
      if (!output_sent_)
      {
        output_sent_ = true;
        activate_ = true;
        init_timeout_ = ros::Time::now();
      }
    }
    if (axes[axis_set_output_] < -0.95)
    {
      if (!output_sent_)
      {
        output_sent_ = true;
        deactivate_ = true;
        init_timeout_ = ros::Time::now();
      }
    }
  }
  else if (buttons[button_dead_man_].isReleased())
  {
  }

  if (output_sent_)
  {
    robotnik_msgs::set_digital_output set_output_srv;
    if (activate_)
    {
      ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Setting outputs...");
      for (int i = 0; i < outputs_.size(); i++)
      {
        set_output_srv.request.output = outputs_[i];
        set_output_srv.request.value = output_values_[i];
      if (set_output_client_.call(set_output_srv) != true || set_output_srv.response.ret != true)
      {
        ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error setting outputs");
      }
      }
      if (checkFeedback(false) || ros::Time::now() - init_timeout_ > ros::Duration(timeout_))
      {
        output_sent_ = false;
        activate_ = false;
      }
    }
    else if (deactivate_)
    {
      ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Setting outputs...");
      for (int i = 0; i < outputs_.size(); i++)
      {
        set_output_srv.request.output = outputs_[i];
        set_output_srv.request.value = !output_values_[i];
        if (set_output_client_.call(set_output_srv) != true || set_output_srv.response.ret != true)
        {
          ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error setting outputs");
        }
      }
      if (checkFeedback(true) || ros::Time::now() - init_timeout_ > ros::Duration(timeout_))
      {
        output_sent_ = false;
        deactivate_ = false;
      }
    }
  }
}

bool PadPluginSetOutput::checkFeedback(bool invert_values)
{
  if (!has_feedback_)
    return true;
  
  bool success = true;

  for (int i = 0; i < inputs_.size(); i++)
  {
    success &= io_.digital_inputs[inputs_[i] - 1] == (input_values_[i] ^ invert_values);
  }
  return success;
}

}  // namespace pad_plugins
