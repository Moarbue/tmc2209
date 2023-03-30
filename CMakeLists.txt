cmake_minimum_required(VERSION 3.5)
idf_component_register(SRCS "src/tmc2209.cpp"
                    INCLUDE_DIRS "src/."
                    REQUIRES "arduino" # Library requires Arduino
                    )

project(tmc2209)