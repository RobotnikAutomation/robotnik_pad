#ifndef PAD_PLUGIN_SET_OUTPUT_H_
#define PAD_PLUGIN_SET_OUTPUT_H_

#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginSetOutput : public GenericPadPlugin
{
public:
  PadPluginSetOutput();
  ~PadPluginSetOutput();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

protected:
  bool checkFeedback(bool invert_values);
  void ioCb(const robotnik_msgs::inputs_outputs::ConstPtr& msg);

  double timeout_;
  int button_dead_man_;
  double axis_set_output_;
  bool output_sent_;
  bool activate_, deactivate_;
  bool use_plc_;
  bool has_feedback_;
  std::vector<int> outputs_;
  std::vector<bool> output_values_;
  std::vector<int> inputs_;
  std::vector<bool> input_values_; 
  std::string base_hw_set_output_name_;
  std::string modbus_set_output_name_;
  std::string base_hw_io_topic_name_;
  std::string modbus_io_topic_name_;
  ros::ServiceClient set_output_client_;
  ros::Subscriber io_sub_;
  robotnik_msgs::inputs_outputs io_;
  ros::Time init_timeout_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_SET_OUTPUT_H_