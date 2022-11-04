#ifndef PAD_PLUGIN_PTZ_H_
#define PAD_PLUGIN_PTZ_H_

#include <robotnik_msgs/ptz.h>
#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad/button.h>


namespace PtzModes
{
  enum PtzMode
  {
      Position = 0,
      Speed = 1
  };
}

namespace pad_plugins
{
class PadPluginPtz : public GenericPadPlugin
{
public:
  PadPluginPtz();
  ~PadPluginPtz();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

  void updateArrows(std::vector<float> &axes);
  void resetReleasedArrows();
  robotnik_msgs::ptz positionControl(const std::vector<Button> &buttons);
  robotnik_msgs::ptz speedControl(const std::vector<Button> &buttons);
  robotnik_msgs::ptz home();
  int ptzMode(const std::vector<Button> &buttons);
  void zoomControl(const std::vector<Button> &buttons);
  void publishPtz(robotnik_msgs::ptz cmd_ptz);

protected:

  int button_dead_man_;
  int button_vertical_arrow_, button_horizontal_arrow_;
  int button_zoom_in_, button_zoom_out_;
  int button_step_up_ , button_step_down_;
  int button_home_;
  int button_ptz_mode_;

  Button up_arrow, down_arrow, left_arrow, right_arrow;

  double tilt_position_, pan_position_, zoom_position_;
  double current_position_increment_;
  double position_increment_;
  double position_increment_limit_;
  double zoom_increment_;
  double min_pan_position_, max_pan_position_;
  double min_tilt_position_, max_tilt_position_;

  double tilt_speed_, pan_speed_;
  double current_speed_;
  double speed_increment_;
  double speed_limit_;

  double zoom_level_;

  double home_pan_position_;
  double home_tilt_position_;
  double home_zoom_position_;

  bool set_position_mode_;
  int ptz_mode_;

  ros::Publisher ptz_pub_;
  std::string cmd_topic_ptz_;

  robotnik_msgs::ptz old_cmd_ptz_;

  double timeout;
  int timeout_counter;
  double last_time;

};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_PTZ_H_