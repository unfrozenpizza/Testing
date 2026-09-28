cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Silvershield-mmWave)

# Add your main application source
target_sources(app PRIVATE src/main.cpp)

# Include the Edge Impulse build logic inside src/
add_subdirectory(src)