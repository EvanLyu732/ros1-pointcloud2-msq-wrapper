message("building sub dir")

add_executable(pointcloud2_sub_node
        pointcloud2_sub_node.cc
)

add_executable(pointcloud2_pub_node
        pointcloud2_pub_node.cc
)

foreach(TARGET pointcloud2_sub_node pointcloud2_pub_node)

    target_link_libraries(${TARGET}
            ${catkin_LIBRARIES}
            ${PROJECT_NAME}
    )

    target_include_directories(${TARGET} PRIVATE
            ${catkin_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/src
    )

endforeach()


