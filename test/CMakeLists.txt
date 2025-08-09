find_package(fmt REQUIRED)
find_package(glog REQUIRED)
find_package(absl REQUIRED)
find_package(CLI11 REQUIRED)

add_executable(bin2pcd bin2pcd.cpp)
target_link_libraries(bin2pcd PRIVATE glog::glog CLI11::CLI11 innolidarsdkcommon)