cmake_minimum_required(VERSION 3.8)

project(kitti-sparsevoxelmap-demo)


# ---------------------------------------------------------------------------
# Set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
# ---------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ".")

# The list of "libs" which can be included can be found in:
#  https://www.mrpt.org/Libraries
# Add the top-level dependencies only.
# --------------------------------------------------------------------------
find_package(mrpt-maps 2.11 REQUIRED)
find_package(mrpt-gui REQUIRED)

find_package(mola_input_kitti_dataset REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mola_metric_maps REQUIRED)

# Define the executable target:
add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
	mrpt::maps
	mrpt::gui
	mola::mola_input_kitti_dataset
	mola::mola_yaml
	mola::mola_metric_maps
)

install(TARGETS ${PROJECT_NAME})

# Set optimized building:
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME}  PRIVATE
		-Wall -pedantic
		-O3
		)
endif()
