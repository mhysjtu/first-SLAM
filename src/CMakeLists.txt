find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

find_package( CSparse REQUIRED )
include_directories( ${CSPARSE_INCLUDE_DIR} )


#==================executable====================
add_executable(testcpp test.cpp)
include_directories("/usr/include/eigen3")
target_link_libraries(testcpp 
	${Sophus_LIBRARIES} 
	${OpenCV_LIBS} 
	g2o_core g2o_stuff
)

add_executable(feature_extraction_orb ch7/feature_extraction_orb.cpp)
target_link_libraries(feature_extraction_orb ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d ch7/pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(triangulation ch7/triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(pose_estimation_2d3d ch7/pose_estimation_2d3d.cpp)
target_link_libraries(pose_estimation_2d3d ${OpenCV_LIBS})

add_executable(pose_estimation_3d3d ch7/pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBS})

add_executable(ceres_curve_fitting ch6/ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting 
	${OpenCV_LIBS} 
	${CERES_LIBRARIES}
)

add_executable(g2o_curve_fitting ch6/g2o_curve_fitting.cpp)
# ${G2O_LIBS} is wrong
target_link_libraries(g2o_curve_fitting 
	${OpenCV_LIBS} 
	g2o_core g2o_stuff
)

add_executable(pose_estimation_2d3d_BA ch7/pose_estimation_2d3d_BA.cpp)
target_link_libraries(pose_estimation_2d3d_BA 
	${OpenCV_LIBS}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
	${CSPARSE_LIBRARY}
)

add_executable(pose_estimation_3d3d_BA ch7/pose_estimation_3d3d_BA.cpp)
target_link_libraries(pose_estimation_3d3d_BA 
	${OpenCV_LIBS}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
	${CSPARSE_LIBRARY}
)
