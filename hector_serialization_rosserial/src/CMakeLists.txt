rosbuild_add_library(hector_serialization_rosserial
  rosserial.cpp
  rosserial/peer.cpp
  rosserial/server.cpp
  rosserial/client.cpp
  rosserial/publisher.cpp
  rosserial/subscriber.cpp
  rosserial/service_server.cpp
  rosserial/service_client.cpp
  rosserial/message_instance.cpp
)

rosbuild_add_executable(serial_server serial_server.cpp)
target_link_libraries(serial_server hector_serialization_rosserial)
rosbuild_link_boost(serial_server system thread)

rosbuild_add_executable(serial_client serial_client.cpp)
target_link_libraries(serial_client hector_serialization_rosserial)
rosbuild_link_boost(serial_client system thread)

rosbuild_add_executable(udp_server udp_server.cpp)
target_link_libraries(udp_server hector_serialization_rosserial)
rosbuild_link_boost(udp_server system thread)

rosbuild_add_executable(udp_client udp_client.cpp)
target_link_libraries(udp_client hector_serialization_rosserial)
rosbuild_link_boost(udp_client system thread)
