# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(
    ESP32_MQTT_Acquisition_Board
    VERSION 1.0
    DESCRIPTION "Low power ESP32 based enviromental data acquisition board, streaming data over MQTT"
    LANGUAGES C CXX
    )
