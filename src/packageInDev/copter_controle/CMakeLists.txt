cmake_minimum_required(VERSION 3.0.2)

project(copter_controle)

# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# 
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)
# 
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp mavros_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# set(PROJECT_SOURCES
#         src/main.cpp
#         src/personal_function.h
#         
# )

# if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#     qt_add_executable(window ${PROJECT_SOURCES})
# else()
#     add_executable(window ${PROJECT_SOURCES})
# endif()

add_executable(copter_controle src/main.cpp)
target_link_libraries(copter_controle PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${catkin_LIBRARIES})
