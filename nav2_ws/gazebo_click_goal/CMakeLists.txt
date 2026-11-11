cmake_minimum_required(VERSION 3.16)
project(ClickToNavigate)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Qt5 REQUIRED COMPONENTS Quick)
add_library(ClickToNavigate SHARED ClickToNavigate.cc ClickToNavigate.qrc)
target_link_libraries(ClickToNavigate PRIVATE gz-gui8::gz-gui8
  gz-rendering8::gz-rendering8 gz-transport13::gz-transport13
  gz-plugin2::register Qt5::Quick)
