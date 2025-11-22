 #  cmake_minimum_required(VERSION 3.5)
#   
#   set(This nop_cpp)
#   
#   project(${This} C CXX)
#   
#   option (BUILD_APP "Build run_to_goal app" ON)
#   option (BUILD_TESTS "Build GTests" ON)
#   option (TO_CATKIN_WS "" OFF)
#   
#   set(CMAKE_C_STANDARD 99)
#   set(CMAKE_CXX_STANDARD 14)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -lboost_program_options")
#   set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#   
#   # set(ONNXRUNTIME_ROOT "/usr/local/onnxruntime") # или куда распаковал
#   set(ONNXRUNTIME_ROOT "/opt/onnxruntime") # или куда распаковал
#   
#   include_directories(${ONNXRUNTIME_ROOT}/include)
#   link_directories(${ONNXRUNTIME_ROOT}/lib)
#   
#   include_directories(
#       lib/include
#       app/include
#   )
#   
#   set(LibSources
#       lib/baseFunctions.cpp
#       lib/controller.cpp
#       lib/model.cpp
#       lib/nop.cpp
#       lib/reader.cpp
#       lib/runner.cpp
#       lib/GANOP.cpp
#   
#       app/RobotSolution.cpp
#       app/simple_solution.cpp
#   )
#   
#   add_library(${This} STATIC ${LibSources})
#   
#   # Линкуем ONNXRuntime
#   target_link_libraries(${This} PUBLIC onnxruntime)
#   
#   if (BUILD_APP)
#   
#   FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
#   INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#   
#   # add_executable(run_to_goal app/run_to_goal.cpp)
#   
#   add_executable(train app/train_robot_control.cpp)
#   
#   add_executable(simple_function app/simple_function.cpp)
#   
#   # target_link_libraries(run_to_goal PUBLIC
#   #     nop_cpp
#   #     ${Boost_LIBRARIES}
#   # )
#   
#   target_link_libraries(train PUBLIC
#       nop_cpp
#       ${Boost_LIBRARIES}
#   )
#   
#   endif()
#   
#   
#   # if (BUILD_TESTS)
#   
#   # add_subdirectory(
#   #     test
#   # )
#   
#   # endif()
#   
#   
#   # if (TO_CATKIN_WS)
#   
#   # install(
#   #     TARGETS nop_cpp
#   #     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   #     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   #     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   # )
#   
#   # endif()
#   
#   

cmake_minimum_required(VERSION 3.5)

set(This nop_cpp)

project(${This} C CXX)

option (BUILD_APP "Build run_to_goal app" ON)
option (BUILD_TESTS "Build GTests" ON)
option (TO_CATKIN_WS "" OFF)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -lboost_program_options")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ONNXRUNTIME_ROOT "/opt/onnxruntime")

include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_ROOT}/lib)

include_directories(
    lib/include
    app/include
)

set(LibSources
    lib/baseFunctions.cpp
    lib/controller.cpp
    lib/model.cpp
    lib/nop.cpp
    lib/reader.cpp
    lib/runner.cpp
    lib/GANOP.cpp
    app/RobotSolution.cpp
    app/simple_solution.cpp
)

add_library(${This} STATIC ${LibSources})

# Линкуем ONNXRuntime
target_link_libraries(${This} PUBLIC onnxruntime)

if (BUILD_APP)

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

add_executable(train app/train_robot_control.cpp)

add_executable(simple_function app/simple_function.cpp)

target_link_libraries(train PUBLIC
    nop_cpp
    ${Boost_LIBRARIES}
)

target_link_libraries(simple_function PUBLIC
    nop_cpp
    ${Boost_LIBRARIES}
)

endif()
