
#pragma once

#include <tf2_ros/message_filter.h>

#include "utils.h"
#include "rclcpp/rclcpp.hpp"

#include <slamware_ros2/msg/vec2_d_int32.h>
#include <slamware_ros2/msg/line2_d_flt32_array.h>

#include "slamware_ros2/msg/rect_int32.h"
#include "slamware_ros2/msg/robot_device_info.h"
#include "slamware_ros2/msg/basic_sensor_info.h"
#include "slamware_ros2/msg/basic_sensor_value.h"
#include "slamware_ros2/msg/robot_basic_state.h"
#include "slamware_ros2/msg/sync_map_request.h"
#include "slamware_ros2/msg/move_by_direction_request.h"
#include "slamware_ros2/msg/move_by_theta_request.h"
#include "slamware_ros2/msg/move_to_request.h"
#include "slamware_ros2/msg/move_to_locations_request.h"
#include "slamware_ros2/msg/map_kind.h"
#include "slamware_ros2/msg/rotate_to_request.h"
#include "slamware_ros2/msg/rotate_request.h"
#include "slamware_ros2/msg/recover_localization_request.h"
#include "slamware_ros2/msg/clear_map_request.h"
#include "slamware_ros2/msg/set_map_update_request.h"
#include "slamware_ros2/msg/set_map_localization_request.h"
#include "slamware_ros2/msg/go_home_request.h"
#include "slamware_ros2/msg/cancel_action_request.h"
#include "slamware_ros2/msg/add_line_request.h"
#include "slamware_ros2/msg/add_lines_request.h"
#include "slamware_ros2/msg/remove_line_request.h"
#include "slamware_ros2/msg/clear_lines_request.h"
#include "slamware_ros2/msg/move_line_request.h"
#include "slamware_ros2/msg/move_lines_request.h"


#include <rpos/core/geometry.h>
#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>

#include <vector>
#include <map>


namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////
    // Important Notes:
    //      Generally, MsgConvert just overwrites known fields;
    //          unknown fields, which are new added and their codes are
    //          not added into MsgConvert, will be unchanged.
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    
    template<class RosMsgT, class SltcValT>
    struct MsgConvert;

    template<class RosMsgT, class SltcValT>
    inline void sltcToRosMsg(const SltcValT& sltcVal, RosMsgT& rosMsg)
    {
        MsgConvert<RosMsgT, SltcValT>::toRos(sltcVal, rosMsg);
    }
    template<class RosMsgT, class SltcValT>
    inline void rosMsgToSltc(const RosMsgT& rosMsg, SltcValT& sltcVal)
    {
        MsgConvert<RosMsgT, SltcValT>::toSltc(rosMsg, sltcVal);
    }

    //////////////////////////////////////////////////////////////////////////

    template<class ValT, class RosMsgT>
    inline void toRosOptionalMsg(const boost::optional<ValT> & optVal, RosMsgT& rosMsg)
    {
        if (optVal)
        {
            rosMsg.is_valid = true;
            sltcToRosMsg(*optVal, rosMsg.value);
        }
        else
        {
            rosMsg = RosMsgT();
        }
    }

    template<class ValT, class RosMsgT>
    inline void fromRosOptionalMsg(const RosMsgT& rosMsg, boost::optional<ValT> & optVal)
    {
        if (rosMsg.is_valid)
        {
            optVal = ValT();
            rosMsgToSltc(rosMsg.value, *optVal);
        }
        else
        {
            optVal.reset();
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<>
    struct MsgConvert<MapKind, rpos::features::location_provider::MapKind>
    {
    public:
        typedef slamware_ros2::msg::MapKind                                        ros_msg_t;
        typedef rpos::features::location_provider::MapKind     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ArtifactUsage, rpos::features::artifact_provider::ArtifactUsage>
    {
    public:
        typedef slamware_ros2::msg::ArtifactUsage                                         ros_msg_t;
        typedef rpos::features::artifact_provider::ArtifactUsage      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<SensorType, rpos::core::SensorType>
    {
    public:
        typedef slamware_ros2::msg::SensorType                  ros_msg_t;
        typedef rpos::core::SensorType      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ImpactType, rpos::features::impact_sensor::ImpactSensorType>
    {
    public:
        typedef slamware_ros2::msg::ImpactType                                          ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorType     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<BasicSensorInfo, rpos::features::impact_sensor::ImpactSensorInfo>
    {
    public:
        typedef slamware_ros2::msg::BasicSensorInfo                                     ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorInfo     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<ActionDirection, rpos::core::ACTION_DIRECTION>
    {
    public:
        typedef slamware_ros2::msg::ActionDirection                     ros_msg_t;
        typedef rpos::core::ACTION_DIRECTION        sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<MoveOptionFlag, rpos::features::motion_planner::MoveOptionFlag>
    {
    public:
        typedef slamware_ros2::msg::MoveOptionFlag                                  ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptionFlag  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<MoveOptions, rpos::features::motion_planner::MoveOptions>
    {
    public:
        typedef slamware_ros2::msg::MoveOptions                                     ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Point, rpos::core::Location>
    {
    public:
        typedef geometry_msgs::msg::Point                ros_msg_t;
        typedef rpos::core::Location                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Quaternion, rpos::core::Rotation>
    {
    public:
        typedef geometry_msgs::msg::Quaternion           ros_msg_t;
        typedef rpos::core::Rotation                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::Pose, rpos::core::Pose>
    {
    public:
        typedef geometry_msgs::msg::Pose                 ros_msg_t;
        typedef rpos::core::Pose                    sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<Vec2DInt32, rpos::core::Vector2i>
    {
    public:
        typedef slamware_ros2::msg::Vec2DInt32                        ros_msg_t;
        typedef rpos::core::Vector2i              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    // Vec2DFlt32 <<======>> rpos::core::Vector2f
    template<>
    struct MsgConvert<Vec2DFlt32, rpos::core::Vector2f>
    {
    public:
        typedef slamware_ros2::msg::Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Vector2f              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };
    // Vec2DFlt32 <<======>> rpos::core::Point
    template<>
    struct MsgConvert<Vec2DFlt32, rpos::core::Point>
    {
    public:
        typedef slamware_ros2::msg::Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Point                 sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<Line2DFlt32, rpos::core::Line>
    {
    public:
        typedef slamware_ros2::msg::Line2DFlt32                       ros_msg_t;
        typedef rpos::core::Line                  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<RectInt32, rpos::core::RectangleI>
    {
    public:
        typedef slamware_ros2::msg::RectInt32                           ros_msg_t;
        typedef rpos::core::RectangleI              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<RectFlt32, rpos::core::RectangleF>
    {
    public:
        typedef slamware_ros2::msg::RectFlt32                           ros_msg_t;
        typedef rpos::core::RectangleF              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<LocalizationMovement, rpos::features::motion_planner::RecoverLocalizationMovement>
    {
    public:
        typedef slamware_ros2::msg::LocalizationMovement                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationMovement     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<LocalizationOptions, rpos::features::motion_planner::RecoverLocalizationOptions>
    {
    public:
        typedef slamware_ros2::msg::LocalizationOptions                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    //////////////////////////////////////////////////////////////////////////

    template<class RosMsgT, class SltcValT, class RosVecAllocT, class SltcVecAllocT>
    struct MsgConvert< std::vector<RosMsgT, RosVecAllocT> , std::vector<SltcValT, SltcVecAllocT> >
    {
    public:
        typedef std::vector<RosMsgT, RosVecAllocT>              ros_msg_t;
        typedef std::vector<SltcValT, SltcVecAllocT>            sltc_val_t;

    public:
        static void toRos(const sltc_val_t& vSltcVal, ros_msg_t& vRosMsg)
        {
            const size_t szCnt = vSltcVal.size();
            vRosMsg.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                sltcToRosMsg(vSltcVal[t], vRosMsg[t]);
            }
        }

        static void toSltc(const ros_msg_t& vRosMsg, sltc_val_t& vSltcVal)
        {
            const size_t szCnt = vRosMsg.size();
            vSltcVal.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                rosMsgToSltc(vRosMsg[t], vSltcVal[t]);
            }
        }
    };

    template<class RosKeyT, class SltcKeyT, class RosMsgT, class SltcValT, class RosCmpT, class SltcCmpT, class RosAllocT, class SltcAllocT>
    struct MsgConvert< std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>, std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT> >
    {
    public:
        typedef std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>              ros_msg_t;
        typedef std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT>          sltc_val_t;

    public:
        static void toRos(const sltc_val_t& mapSltcVal, ros_msg_t& mapRosMsg)
        {
            mapRosMsg.clear();
            for (auto cit = mapSltcVal.cbegin(), citEnd = mapSltcVal.cend(); citEnd != cit; ++cit)
            {
                sltcToRosMsg(cit->second, mapRosMsg[cit->first]);
            }
        }

        static void toSltc(const ros_msg_t& mapRosMsg, sltc_val_t& mapSltcVal)
        {
            mapSltcVal.clear();
            for (auto cit = mapRosMsg.cbegin(), citEnd = mapRosMsg.cend(); citEnd != cit; ++cit)
            {
                rosMsgToSltc(cit->second, mapSltcVal[cit->first]);
            }
        }
    };

    //////////////////////////////////////////////////////////////////////////
    
}
