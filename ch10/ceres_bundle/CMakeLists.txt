cmake_minimum_required(VERSION 2.8)
project(ceres_bundle)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS}
					${EIGEN_INCLUDE_DIRS} 
                    ${PROJECT_SOURCE_DIR}/common
                    ${PROJECT_SOURCE_DIR}/common/tools 
                    ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

add_executable(ceres_bundle src/ceresBundle.cpp)
target_link_libraries(ceres_bundle BALProblem ParseCmd ${CERES_LIBRARIES})
