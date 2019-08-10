find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(testcpp test.cpp)
include_directories("/usr/include/eigen3")
target_link_libraries(testcpp ${Sophus_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBS})

add_executable(feature_extraction_orb ch7/feature_extraction_orb.cpp)
target_link_libraries(feature_extraction_orb ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d ch7/pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})
