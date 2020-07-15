
add_library(rslidar_point convert.cc)
target_link_libraries(rslidar_point
    ${catkin_LIBRARIES})


add_library(cloud_nodelet cloud_nodelet.cc)
target_link_libraries(cloud_nodelet rslidar_point
                      ${catkin_LIBRARIES})

add_executable(cloud_node cloud_node.cc)
target_link_libraries(cloud_node
    rslidar_point
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
