# AES70 (OCA) DSP device demo.
cmake_minimum_required(VERSION 3.16)

# Pull in the aes70 component (two levels up) and the IDF example connection
# helper (protocol_examples_common, provides example_connect()).
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "$ENV{IDF_PATH}/examples/common_components/protocol_examples_common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(aes70_dsp_demo)
