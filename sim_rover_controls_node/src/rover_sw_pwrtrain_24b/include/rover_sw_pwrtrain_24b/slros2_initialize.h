// Copyright 2022-2024 The MathWorks, Inc.
// Generated 31-Jan-2026 01:10:14
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
// rover_sw_pwrtrain_24b/ros_publish/Publish10
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1122;
// rover_sw_pwrtrain_24b/ros_publish/Publish2
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_826;
// rover_sw_pwrtrain_24b/ros_publish/Publish3
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_246;
// rover_sw_pwrtrain_24b/ros_publish/Publish4
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_1006;
// rover_sw_pwrtrain_24b/ros_publish/Publish5
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_903;
// rover_sw_pwrtrain_24b/ros_publish/Publish6
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_1009;
// rover_sw_pwrtrain_24b/ros_publish/Publish7
extern SimulinkPublisher<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Pub_rover_sw_pwrtrain_24b_1039;
// rover_sw_pwrtrain_24b/ros_publish/Publish8
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1087;
// rover_sw_pwrtrain_24b/ros_publish/Publish9
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1115;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish1
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1057;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish2
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1058;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish3
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1059;
// rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish4
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_rover_sw_pwrtrain_24b_1060;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_390;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_391;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe10
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_1130;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe11
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_1131;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe12
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_1132;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe13
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_1133;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_392;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe3
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_986;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4
extern SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_rover_sw_pwrtrain_24b_521;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe5
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_987;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe6
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_988;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe7
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_989;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe8
extern SimulinkSubscriber<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Sub_rover_sw_pwrtrain_24b_1018;
// rover_sw_pwrtrain_24b/ros_subscribe/Subscribe9
extern SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_rover_sw_pwrtrain_24b_1042;
#endif
