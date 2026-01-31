#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include "rover_sw_pwrtrain_24b_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(std_msgs::msg::Bool& msgPtr, SL_Bus_std_msgs_Bool const* busPtr);
void convertToBus(SL_Bus_std_msgs_Bool* busPtr, const std_msgs::msg::Bool& msgPtr);

void convertFromBus(std_msgs::msg::Float32& msgPtr, SL_Bus_std_msgs_Float32 const* busPtr);
void convertToBus(SL_Bus_std_msgs_Float32* busPtr, const std_msgs::msg::Float32& msgPtr);

void convertFromBus(std_msgs::msg::Int32& msgPtr, SL_Bus_std_msgs_Int32 const* busPtr);
void convertToBus(SL_Bus_std_msgs_Int32* busPtr, const std_msgs::msg::Int32& msgPtr);


#endif
