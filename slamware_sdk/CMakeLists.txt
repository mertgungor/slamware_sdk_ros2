cmake_minimum_required(VERSION 3.5)
project(slamware_sdk)

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/lib)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
    set(slamware_sdk_FOUND true)
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
    set(slamware_sdk_FOUND false)
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)


add_library(slamware_sdk INTERFACE)
target_include_directories(slamware_sdk INTERFACE ${slamware_sdk_INCLUDE_DIR})
target_link_libraries(slamware_sdk INTERFACE ${slamware_sdk_LIBRARY})

install(TARGETS slamware_sdk
        EXPORT export_${PROJECT_NAME}
        INCLUDES DESTINATION include
)
install(DIRECTORY ${slamware_sdk_INCLUDE_DIR}/ DESTINATION include)

