find_package(Threads REQUIRED)

include(FindPackageHandleStandardArgs)
set(${CMAKE_FIND_PACKAGE_NAME}_CONFIG ${CMAKE_CURRENT_LIST_FILE})
find_package_handle_standard_args(nadjieb_mjpeg_streamer CONFIG_MODE)

if(NOT TARGET nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)
    include("${CMAKE_CURRENT_LIST_DIR}/nadjieb_mjpeg_streamerTargets.cmake")
    if((NOT TARGET nadjieb_mjpeg_streamer) AND
       (NOT nadjieb_mjpeg_streamer_FIND_VERSION OR
        nadjieb_mjpeg_streamer_FIND_VERSION VERSION_LESS 3.2.0))
        add_library(nadjieb_mjpeg_streamer INTERFACE IMPORTED)
        set_target_properties(nadjieb_mjpeg_streamer PROPERTIES
            INTERFACE_LINK_LIBRARIES nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer
        )
    endif()
endif()
