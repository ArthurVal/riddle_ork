cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)

 option(LINEMOD_VISUALIZE_IMAGE "visualize images with detected objects features" OFF)
option(LINEMOD_VISUALIZE_POINTCLOUD "visualize point clouds from the reference image and its match from DB" OFF)

find_package(catkin REQUIRED object_recognition_core object_recognition_renderer opencv_candidate)
catkin_package(CATKIN_DEPENDS ecto object_recognition_core object_recognition_renderer opencv_candidate
               DEPENDS OpenCV
)

#install targets for all things python
catkin_python_setup()

find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# Compile the module
add_subdirectory(python)
add_subdirectory(src)

# add the testsls
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# build the docs
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the conf files
install(DIRECTORY conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
