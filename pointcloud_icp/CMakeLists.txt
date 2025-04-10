cmake_minimum_required(VERSION 3.5)
project(PointCloudICP)

# Find PCL and VTK
find_package(PCL REQUIRED COMPONENTS common io registration visualization)
find_package(VTK REQUIRED)

# Include paths
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Main ICP program
add_executable(icp main.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})

# Visualization program
add_executable(visualize visualize.cpp)
target_link_libraries(visualize ${PCL_LIBRARIES} ${VTK_LIBRARIES})
