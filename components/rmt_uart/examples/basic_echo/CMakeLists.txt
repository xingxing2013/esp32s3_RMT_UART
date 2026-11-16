cmake_minimum_required(VERSION 3.16)

# 三个组件的根目录：components/
set(EXTRA_COMPONENT_DIRS "../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rmt_uart_basic_echo)
