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
  readParam(pnh_, "config/button_set_output", button_set_output_, button_set_output_, required);
  readParam(pnh_, "config/button_reset_output", button_reset_output_, button_reset_output_, required);
  readParam(pnh_, "use_buttons", use_buttons_, false, not_required);
  readParam(pnh_, "base_hw_set_output_name", base_hw_set_output_name_, base_hw_set_output_name_, required);
  readParam(pnh_, "modbus_set_output_name", modbus_set_output_name_, modbus_set_output_name_, required);
  readParam(pnh_, "use_plc", use_plc_, false, required);
  readParam(pnh_, "has_feedback", has_feedback_, false, not_required);
  readParam(pnh_, "has_condition", has_condition_, false, not_required);
  timeout_ = 1;
  readParam(pnh_, "timeout", timeout_, timeout_, not_required);
  if (use_plc_)
    feedback_input_topic_name_ = "robotnik_modbus_io/inputs_outputs";
  else
    feedback_input_topic_name_ = "robotnik_base_hw/io";
  condition_input_topic_name_ = feedback_input_topic_name_;

  readParam(pnh_, "feedback_input_topic_name", feedback_input_topic_name_, feedback_input_topic_name_, not_required);
  readParam(pnh_, "condition_input_topic_name", condition_input_topic_name_, condition_input_topic_name_, not_required);

  getIoValues(pnh_, "outputs", outputs_);
  if (has_feedback_)
  {
    getIoValues(pnh_, "feedback_inputs", inputs_);
    if (inputs_.size() == 0)
      has_feedback_ = false;
  }
  if (has_condition_)
  {
    getIoValues(pnh_, "condition_inputs", condition_inputs_);
    if (condition_inputs_.size() == 0)
      has_condition_ = false;
  }

  // Service client
  std::string service_name = use_plc_ ? modbus_set_output_name_ : base_hw_set_output_name_;
  set_output_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(service_name);

  //Subscriber
  if (has_feedback_ || has_condition_)
  {
    if (feedback_input_topic_name_ == condition_input_topic_name_ && has_feedback_ && has_condition_)
    {
      io_sub_ = nh_.subscribe<robotnik_msgs::inputs_outputs>(feedback_input_topic_name_, 1, &PadPluginSetOutput::ioCb, this);
    }
    else
    {
      if (has_feedback_)
      {
        feedback_io_sub_ = nh_.subscribe<robotnik_msgs::inputs_outputs>(feedback_input_topic_name_, 1, &PadPluginSetOutput::feedbackIoCb, this);
      }
      if (has_condition_)
      {
        condition_io_sub_ = nh_.subscribe<robotnik_msgs::inputs_outputs>(condition_input_topic_name_, 1, &PadPluginSetOutput::conditionIoCb, this);
      }
    }
  }
}

void PadPluginSetOutput::getIoValues(ros::NodeHandle& nh, const std::string& param_name, std::vector<io>& variable)
{
  XmlRpc::XmlRpcValue in_out;
  nh.getParam(param_name, in_out);
  int i;
  io aux_in_out;
  if (in_out.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (i = 0; i < in_out.size(); i++)
    {
      aux_in_out.number = in_out[i]["number"];
      aux_in_out.value = in_out[i]["value"];
      variable.push_back(aux_in_out);
    }
  }
  else if (in_out.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    aux_in_out.number = in_out["number"];
    aux_in_out.value = in_out["value"];
    variable.push_back(aux_in_out);
  }
  else
  {
    ROS_ERROR("Parameter %s does not contain a list or a struct with int 'number' and bool 'value'.", param_name.c_str());
  }
}

void PadPluginSetOutput::ioCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg)
{
  io_ = *msg;
  condition_io_ = *msg;
}

void PadPluginSetOutput::feedbackIoCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg)
{
  io_ = *msg;
}

void PadPluginSetOutput::conditionIoCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg)
{
  condition_io_ = *msg;
}

void PadPluginSetOutput::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if ((axes[axis_set_output_] > 0.95 && !use_buttons_) || (buttons[button_set_output_].isReleased() && use_buttons_))
    {
      if (!checkCondition())
      {
        ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Conditions not met for output activation");
      }
      else if (!output_sent_)
      {
        ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Setting outputs...");
        output_sent_ = true;
        activate_ = true;
        init_timeout_ = ros::Time::now();
      }
    }
    if ((axes[axis_set_output_] < -0.95  && !use_buttons_) || (buttons[button_reset_output_].isReleased() && use_buttons_))
    {
      if (!checkCondition())
      {
        ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Conditions not met for output deactivation");
      }
      else if (!output_sent_)
      {
        ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Resetting outputs...");
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
      for (int i = 0; i < outputs_.size(); i++)
      {
        set_output_srv.request.output = outputs_[i].number;
        set_output_srv.request.value = outputs_[i].value;
        if (set_output_client_.call(set_output_srv) != true || set_output_srv.response.ret != true)
        {
          ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error setting outputs");
        }
      }
      if (checkFeedback(false))
      {
        output_sent_ = false;
        activate_ = false;
        ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Outputs correctly set");
      }
      else if (ros::Time::now() - init_timeout_ > ros::Duration(timeout_))
      {
        output_sent_ = false;
        activate_ = false;
        ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error setting outputs after %.2f seconds", timeout_);
      }
    }
    else if (deactivate_)
    {
      for (int i = 0; i < outputs_.size(); i++)
      {
        set_output_srv.request.output = outputs_[i].number;
        set_output_srv.request.value = !outputs_[i].value;
        if (set_output_client_.call(set_output_srv) != true || set_output_srv.response.ret != true)
        {
          ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error resetting outputs");
        }
      }
      if (checkFeedback(true))
      {
        output_sent_ = false;
        deactivate_ = false;
        ROS_INFO_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Outputs correctly reset");
      }
      else if (ros::Time::now() - init_timeout_ > ros::Duration(timeout_))
      {
        output_sent_ = false;
        deactivate_ = false;
        ROS_ERROR_NAMED("PadPluginSetOutput", "PadPluginSetOutput::execute: Error resetting outputs after %.2f seconds", timeout_);
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
    success &= io_.digital_inputs[inputs_[i].number - 1] == (inputs_[i].value ^ invert_values);
  }
  return success;
}

bool PadPluginSetOutput::checkCondition()
{
  if (!has_condition_)
    return true;

  bool success = true;
  for (int i = 0; i < condition_inputs_.size(); i++)
  {
    success &= (condition_io_.digital_inputs[condition_inputs_[i].number - 1] == condition_inputs_[i].value);
  }
  return success;
}

}  // namespace pad_plugins
