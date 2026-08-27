set(ZP_PLATFORM_SYSTEM_LAYER linux)
set(ZP_PLATFORM_COMPILE_DEFINITIONS ZENOH_LINUX)
set(ZP_PLATFORM_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/system/unix/system.c"
    "${PROJECT_SOURCE_DIR}/src/system/unix/network.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/udp/raweth_unix.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/tcp/tcp_posix.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/udp/udp_posix.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/serial/tty_posix.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/can/can_socketcan.c"
    "${PROJECT_SOURCE_DIR}/src/link/transport/isotp/isotp_socketcan.c")
if(ZP_UDP_MULTICAST_ENABLED)
  list(APPEND ZP_PLATFORM_SOURCE_FILES
       "${PROJECT_SOURCE_DIR}/src/link/transport/udp/udp_multicast_posix.c")
endif()
