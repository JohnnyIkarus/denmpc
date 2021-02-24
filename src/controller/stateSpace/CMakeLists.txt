find_package(ur_rtde REQUIRED)
add_library(stateSpace SimulateSystem.cpp SimulateSystem.h)
target_link_libraries(stateSpace PRIVATE ur_rtde::rtde)