cmake_minimum_required(VERSION 2.8.3)
project(srrt_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            dynamic_reconfigure
            pluginlib
        )

include_directories(
  include ${catkin_INCLUDE_DIRS}
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/SrrtPlanner.cfg
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES srrt_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        dynamic_reconfigure
        nav_core
)

add_library(srrt_planner
  src/srrt_vertex.cpp
  include/srrt_planner/srrt_vertex.h src/srrt_planner.cpp)
add_dependencies(srrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(srrt_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS srrt_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


