#include <robotnik_pad/robotnik_pad.h>

RobotnikPad::RobotnikPad() : Node("robotnik_pad")
{
    rosReadParams();
    rosSetup();
}

void RobotnikPad::start()
{
    setup();
    
    double control_loop_period = 1.0 / desired_freq_;
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_loop_period), std::bind(&RobotnikPad::controlLoop, this)
    );
}

void RobotnikPad::rosReadParams()
{   
    RobotnikPad::readParam("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ, false);
    if (desired_freq_ <= 0)
    {
        desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;
    }

    uint8_t default_num_of_butons = 0;
    RobotnikPad::readParam("pad.num_of_buttons", num_of_buttons_, default_num_of_butons, true);
    uint8_t default_num_of_axes = 0;
    RobotnikPad::readParam("pad.num_of_axes", num_of_axes_, default_num_of_axes, true);
    std::string default_joy_topic_name = "joy";
    RobotnikPad::readParam("pad.joy_topic", joy_topic_, default_joy_topic_name, false);
    RobotnikPad::readParam("pad.joy_timeout", joy_timeout_, 5.0, false);

    std::vector<std::string> plugin_names;
    this->declare_parameter("plugins", std::vector<std::string>());
    this->get_parameter("plugins", plugin_names);
    readPluginsFromParams(plugin_names, plugins_from_params_);
}

void RobotnikPad::setup()
{
    for (int i = 0; i < num_of_buttons_; i++)
    {
        buttons_.push_back(Button());
    }

    for (int i = 0; i < num_of_axes_; i++)
    {
        axes_.push_back(Axes());
    }

    pad_plugins_loader_ = new pluginlib::ClassLoader<pad_plugins::GenericPadPlugin>("robotnik_pad", "pad_plugins::" "GenericPadPlugin");

    for (auto& param_plugin : plugins_from_params_)
    {
        std::shared_ptr<pad_plugins::GenericPadPlugin> plugin;
        try
        {
            plugin = pad_plugins_loader_->createSharedInstance(param_plugin.second);
            
        }
        catch (pluginlib::PluginlibException& ex)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load plugin " << param_plugin.first << "\" of type \"" << param_plugin.second
                                                    << "." << std::endl
                                                    << "Exception: " << ex.what());
            continue;
        }
        
        plugin->initialize(this->shared_from_this(), param_plugin.first);
        plugins_.push_back(plugin);
    }
}

void RobotnikPad::rosSetup()
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 1, std::bind(&RobotnikPad::joyCb, this, std::placeholders::_1));
    
    joy_topic_last_time_received_ = this->now();
}

void RobotnikPad::readPluginsFromParams(const std::vector<std::string>& names, std::map<std::string, std::string>& plugins_definitions)
{
    plugins_definitions.clear();
    for (const std::string& name : names)
    {
        this->declare_parameter(name, "");
        if (!this->has_parameter(name))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot load component " << name << " because it doesn't exist in the parameter tree");
            continue;
        }

        std::string type = "";
        
        bool param_read = false;
        param_read = RobotnikPad::readParam(name + ".type", type, type, true);
        if (!param_read)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot load component " << name << " because its type is empty");
            continue;
        }
        if (plugins_definitions.count(name) != 0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Already loaded component with name " << name << " of type " << type);
        }
        plugins_definitions[name] = type;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "I have read " << plugins_definitions.size() << " components:");
    for (auto& definition : plugins_definitions)
        RCLCPP_INFO_STREAM(this->get_logger(), definition.first << ": " << definition.second);
}

void RobotnikPad::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    for (unsigned int i = 0; i < msg->buttons.size(); i++)
    {
        buttons_[i].press(msg->buttons[i]);
    }

    for (unsigned int i = 0; i < msg->axes.size(); i++)
    {
        axes_[i].press(msg->axes[i]);
    }

    joy_topic_last_time_received_ = this->now();
}

void RobotnikPad::controlLoop()
{
    if ((this->now() - joy_topic_last_time_received_).seconds() > joy_timeout_)
    {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Topic " << joy_topic_ << " is not being received");
    }
    else
    {
        for (auto& plugin : plugins_)
        {
            plugin->execute(buttons_, axes_);
        }

        for (auto& button : buttons_)
        {
            button.resetReleased();
        }

        for (auto& axis : axes_)
        {
            axis.resetReleased();
        }
    }
}