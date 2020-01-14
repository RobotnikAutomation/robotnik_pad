#include <pluginlib/class_list_macros.h>

#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad_plugins/movement_plugin.h>

PLUGINLIB_EXPORT_CLASS(pad_plugins::PadPluginMovement, pad_plugins::GenericPadPlugin);