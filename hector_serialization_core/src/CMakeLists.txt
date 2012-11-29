rosbuild_add_library(hector_serialization
  channel.cpp
  device/file.cpp
  device/serial_port.cpp
  socket/udp.cpp
  socket/tcp.cpp
  socket/context.cpp
)
rosbuild_link_boost(hector_serialization system thread)
