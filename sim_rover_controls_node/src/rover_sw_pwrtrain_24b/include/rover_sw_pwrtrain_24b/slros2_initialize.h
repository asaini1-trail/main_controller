// Copyright 2022-2024 The MathWorks, Inc.
// Generated 11-Jan-2026 12:00:11
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "rover_sw_pwrtrain_24b_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, _history, _depth, _durability, _reliability, _deadline \
, _lifespan, _liveliness, _lease_duration, _avoid_ros_namespace_conventions)             \
    {                                                                                    \
        qosStruct.history = _history;                                                    \
        qosStruct.depth = _depth;                                                        \
        qosStruct.durability = _durability;                                              \
        qosStruct.reliability = _reliability;                                            \
        qosStruct.deadline.sec = _deadline.sec;                                          \
        qosStruct.deadline.nsec = _deadline.nsec;                                        \
        qosStruct.lifespan.sec = _lifespan.sec;                                          \
        qosStruct.lifespan.nsec = _lifespan.nsec;                                        \
        qosStruct.liveliness = _liveliness;                                              \
        qosStruct.liveliness_lease_duration.sec = _lease_duration.sec;                   \
        qosStruct.liveliness_lease_duration.nsec = _lease_duration.nsec;                 \
        qosStruct.avoid_ros_namespace_conventions = _avoid_ros_namespace_conventions;    \
    }
#endif
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}
// rover_sw_pwrtrain_24b/ros_publish/Publish1
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_907;
// rover_sw_pwrtrain_24b/ros_publish/Publish2
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_826;
// rover_sw_pwrtrain_24b/ros_publish/Publish3
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_246;
// rover_sw_pwrtrain_24b/ros_publish/Publish5
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_903;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Publish1
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_542;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Publish2
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_543;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Publish1
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_138;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Publish2
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_139;
// rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish1
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_818;
// rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish2
extern SimulinkPublisher<std_msgs::msg::Float32MultiArray,SL_Bus_std_msgs_Float32MultiArray> Pub_rover_sw_pwrtrain_24b_822;
// rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish4
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_835;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_390;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_391;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_392;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4
extern SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_rover_sw_pwrtrain_24b_521;
#endif
