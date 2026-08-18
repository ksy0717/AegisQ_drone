# Project build profile for the long-lived MQTT/PX4 control branch.
#
# This is the single source of default build options. Command-line -D values
# still win because these are cache entries without FORCE.
set(DC_PROJECT_VARIANT "mqtt-px4-control" CACHE STRING
    "Name reported in the CMake build summary")

set(DC_BUILD_VIDEO ON CACHE BOOL
    "Build the camera/display/RTSP/WHIP process")
set(DC_BUILD_VIDEO_TESTS ON CACHE BOOL
    "Build the GStreamer-free video topology and WHIP configuration tests")
set(DC_INSTALL_GST_WHIP_PLUGIN ON CACHE BOOL
    "Install the pinned WHIP runtime plugin on Raspberry Pi")
set(DC_HAILORT_MODE "OFF" CACHE STRING
    "Compile HailoRT support: AUTO, ON, or OFF")
set_property(CACHE DC_HAILORT_MODE PROPERTY STRINGS AUTO ON OFF)

set(DC_BUILD_FLIGHT_CORE ON CACHE BOOL
    "Expose the transport-neutral flight contract and shared Riposte math")
set(DC_BUILD_RIPOSTE_AGENT OFF CACHE BOOL
    "Build the tablet-commanded Riposte vehicle agent")
set(DC_BUILD_RIPOSTE_GCS OFF CACHE BOOL
    "Build the Riposte ground-side command tool")

# The inbound connector is a separately testable module. It does not own PX4.
set(DC_BUILD_MQTT_CONNECTOR ON CACHE BOOL
    "Build the strict AegisQ MQTT command codec and bounded CommandSource")
set(DC_BUILD_MQTT_TRANSPORT ON CACHE BOOL
    "Build the Linux libmosquitto mTLS/WireGuard adapter")
set(DC_BUILD_MQTT_PERSISTENCE ON CACHE BOOL
    "Build the SQLite command ledger and durable ACK outbox")

# Kept OFF until durable inbox/outbox/UUID deduplication, priority
# STOP/RTL/E-STOP, terminal status publication, the MAVSDK executor adapter,
# and SITL/vehicle verification land. An incomplete executable must never be
# presented as flight-ready.
set(DC_BUILD_MQTT_PX4_AGENT OFF CACHE BOOL
    "Build the web-server MQTT to PX4 control agent")

set(DC_BUILD_FLIGHT_CORE_TESTS ON CACHE BOOL
    "Build dependency-free shared flight tests")
set(DC_BUILD_REMOTE_COMMAND_TESTS ON CACHE BOOL
    "Build MQTT/PX4 command-contract tests")
set(DC_BUILD_MQTT_CONNECTOR_TESTS ON CACHE BOOL
    "Build AegisQ JSON codec and MQTT command-source tests")
set(DC_BUILD_MQTT_PERSISTENCE_TESTS ON CACHE BOOL
    "Build SQLite command ledger and ACK outbox tests")
set(DC_BUILD_RIPOSTE_TESTS OFF CACHE BOOL
    "Build Riposte protocol and verdict tests")

# scripts/install.sh queries dependency requirements through CMake script mode
# so this file remains the only source of build-selection truth.
if(DC_PRINT_PROFILE)
    if(DC_BUILD_RIPOSTE_AGENT OR DC_BUILD_RIPOSTE_GCS OR DC_BUILD_MQTT_PX4_AGENT)
        message(STATUS "DC_PROFILE_NEEDS_MAVSDK=ON")
    else()
        message(STATUS "DC_PROFILE_NEEDS_MAVSDK=OFF")
    endif()
    if(DC_BUILD_MQTT_CONNECTOR OR DC_BUILD_MQTT_TRANSPORT
       OR DC_BUILD_MQTT_PERSISTENCE)
        message(STATUS "DC_PROFILE_NEEDS_MQTT=ON")
    else()
        message(STATUS "DC_PROFILE_NEEDS_MQTT=OFF")
    endif()
    if(DC_INSTALL_GST_WHIP_PLUGIN)
        message(STATUS "DC_PROFILE_NEEDS_GST_WHIP=ON")
    else()
        message(STATUS "DC_PROFILE_NEEDS_GST_WHIP=OFF")
    endif()
endif()
