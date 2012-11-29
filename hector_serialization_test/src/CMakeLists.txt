rosbuild_add_executable(xbee xbee.cpp)
rosbuild_link_boost(xbee thread system)

rosbuild_add_executable(udp udp.cpp)
rosbuild_link_boost(udp thread system)
