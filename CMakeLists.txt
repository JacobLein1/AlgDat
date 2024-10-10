cmake_minimum_required(VERSION 3.29)
project(Ovinger C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

# Oving3 executable
add_executable(oving3
        oving3.c
)

# Oving4_1 executable
add_executable(oving4_1
        oving4/oving4_1.c
)

# Oving4_2 executable
add_executable(oving4_2
        oving4/oving4_2.c
)
add_executable(oving5
        oving5/oving5.c
)
