cmake_minimum_required(VERSION 3.16)
project(Neo_Exos)

set(CMAKE_CXX_STANDARD 20)

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

#LPMS_IMU
INCLUDE_DIRECTORIES(./LpSensor-1.3.5-Linux-x86-64/include)
LINK_DIRECTORIES(./LpSensor-1.3.5-Linux-x86-64/lib)

add_executable(Neo_Exos main.cpp exos.h exos.cpp ButterWorthLP.cpp ButterWorthLP.h dataDecode.h dynamics.h PID.h traj_generate.h traj_generate.cpp Utils.h Unit.h timeInterval.cpp timeInterval.h Period.h Period.cpp)

target_link_libraries(${PROJECT_NAME} ethercat pthread)
#LPMS_IMU
target_link_libraries(${CMAKE_PROJECT_NAME} LpSensor dl)