cmake_minimum_required(VERSION 3.10)
project(unitree_motor_control)

set(CMAKE_BUILD_TYPE Debug) # Debug/Release

# g++编译选项 -O3优化代码
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++14 -Wall")

# using pkg-config find pinocchio
set(ENV{PKG_CONFIG_PATH} /opt/openrobots/lib/pkgconfig)
find_package(PkgConfig REQUIRED) 
message(STATUS "=== PIN_LIBRARIES: ${PIN_LIBRARIES}") # 打印库信息
message(STATUS "=== PIN_INCLUDE_DIRS: ${PIN_INCLUDE_DIRS}") # 打印include路径信息
set(PINOCCHINO_LIB_DIR "/opt/openrobots/lib")
pkg_check_modules(PIN REQUIRED pinocchio) # PIN 

# unitree 
set(UNITREE_LIBS libunitreeMotorSDK_Linux64.so)

# 添加头文件路径 -I
include_directories(${CMAKE_SOURCE_DIR}/include    
                    /usr/include/eigen3            # Eigen
)

# 添加源文件路径
set(SRC 
    src/motor_control.cpp
    src/periodic_rt_task.cpp
    # src/traj_generator.cpp
) 

# 添加链接库文件路径 -L
link_directories(lib                 # unitree A1 motor 
                PINOCCHINO_LIB_DIR # pinocchio
)

# # 输出可执行文件 
# add_executable(single_leg src/leg_test/single_leg.cpp ${SRC})
# # 链接库
# target_link_libraries(single_leg PRIVATE 
#                     ${UNITREE_LIBS}   # unitree motor 
#                     pthread           # C thread 
#                     ${PIN_LIBRARIES}  # pinocchio
# )
# target_include_directories(single_leg PRIVATE ${PIN_INCLUDE_DIRS})
# target_compile_options(single_leg PRIVATE ${PIN_CFLAGS_OTHER})


# # 输出可执行文件 
# add_executable(multi_leg src/leg_test/multi_leg.cpp ${SRC})
# # 链接库
# target_link_libraries(multi_leg PRIVATE 
#                     ${UNITREE_LIBS}   # unitree motor 
#                     pthread           # C thread 
#                     ${PIN_LIBRARIES}  # pinocchio
# )
# target_include_directories(multi_leg PRIVATE ${PIN_INCLUDE_DIRS})
# target_compile_options(multi_leg PRIVATE ${PIN_CFLAGS_OTHER})

# # 输出可执行文件 
# add_executable(rt_control src/motor_test/rt_control.cpp ${SRC})
# # 链接库 -l
# target_link_libraries(rt_control PRIVATE 
#                     ${UNITREE_LIBS}   # unitree motor 
#                     pthread           # C thread 
# )     

# # 输出可执行文件 
# add_executable(multi_thread src/motor_test/rt_control_multi_thread.cpp ${SRC})
# # 链接库
# target_link_libraries(multi_thread PRIVATE 
#                     ${UNITREE_LIBS}   # unitree motor 
#                     pthread           # C thread 
# )         

# # 输出可执行文件 
# add_executable(multi_thread2 src/motor_test/rt_control_multi_thread2.cpp ${SRC})
# # 链接库
# target_link_libraries(multi_thread2 PRIVATE 
#                     ${UNITREE_LIBS}   # unitree motor 
#                     pthread           # C thread 
# )         

# 输出可执行文件 
add_executable(calibration src/tools/calibration.cpp ${SRC})
# 链接库
target_link_libraries(calibration PRIVATE 
                    ${UNITREE_LIBS}   # unitree motor 
                    pthread           # C thread 
)         

# 输出可执行文件 
add_executable(changeID src/tools/changeID.cpp)
target_link_libraries(changeID ${UNITREE_LIBS})

# 可执行文件路径 
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
