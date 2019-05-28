cmake_minimum_required(VERSION 2.8.3)
project(dstarl_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
            global_planner
            costmap_2d
            pluginlib
        )

include_directories(
  include ${catkin_INCLUDE_DIRS}
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dstarl_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        nav_core
        global_planner
)

add_library(dstarl_planner  src/dstarl_planner.cpp include/dstarl_planner/dstarl_planner.h)
add_dependencies(dstarl_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dstarl_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS dstarl_planner
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