#ifndef PAD_PLUGIN_SET_OUTPUT_H_
#define PAD_PLUGIN_SET_OUTPUT_H_

#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{

struct io {
  int number;
  bool value;
};
class PadPluginSetOutput : public GenericPadPlugin
{
public:
  PadPluginSetOutput();
  ~PadPluginSetOutput();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

protected:
  bool checkFeedback(bool invert_values);
  bool checkCondition();
  void ioCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg);
  void feedbackIoCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg);
  void conditionIoCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg);
  void getIoValues(ros::NodeHandle& nh, const std::string& param_name, std::vector<io>& variable);

  double timeout_;
  int button_dead_man_;
  double axis_set_output_;
  bool output_sent_;
  bool activate_, deactivate_;
  bool use_plc_;
  bool has_feedback_;
  bool has_condition_;
  bool use_buttons_;
  int button_set_output_;
  int button_reset_output_;
  std::vector<io> outputs_;
  std::vector<io> inputs_;
  std::vector<io> condition_inputs_;
  std::string base_hw_set_output_name_;
  std::string modbus_set_output_name_;
  std::string feedback_input_topic_name_;
  std::string condition_input_topic_name_;
  ros::ServiceClient set_output_client_;
  ros::Subscriber io_sub_, feedback_io_sub_, condition_io_sub_;
  robotnik_msgs::inputs_outputs io_, condition_io_;
  ros::Time init_timeout_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_SET_OUTPUT_H_