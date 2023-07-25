cmake_minimum_required(VERSION 3.25)
project(T310_libiio)

set(CMAKE_CXX_STANDARD 17)




add_executable(T310_libiio main.cpp antsdrDevice.h antsdrDevice.cpp)

target_link_libraries(T310_libiio iio pthread)
