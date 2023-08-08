#pragma once

#include <rpos/core/rpos_core_config.h>

#include <cstdint>

namespace rpos { namespace core {

    struct RPOS_CORE_API SlamcoreShutdownParam
    {
        std::uint32_t restartTimeIntervalMinute;
        std::uint32_t shutdownTimeIntervalMinute;
        bool resetShutdownRestartTimeInterval;

        SlamcoreShutdownParam();
    };

    struct RPOS_CORE_API SetChargeControl
    {
        std::uint32_t chargeControl;    //=1: disable charging; =0: enable charging

        SetChargeControl();
    };

    struct RPOS_CORE_API GetChargeControlStatus
    {
        std::uint32_t chargeControlStatus;  //=1: disabled charging; =0: enabled charging

        GetChargeControlStatus();
    };

}}
