cmake_minimum_required(VERSION 3.1)
project(bim_data)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )

#Data folder
add_definitions(-DTEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    set(CGAL_ADD_LIB gmpxx)
    add_definitions(-DCGAL_USE_GMPXX)
endif (OPENMP_FOUND)

find_package(HDF5 REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS})

# Tinyply
add_library(tinyply external/tinyply/tinyply.cpp)

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Widgets Core Xml OpenGL REQUIRED)

# CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
message("CGAL_VERSION: ${CGAL_VERSION} test")
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CGAL_INCLUDE_DIRS})
#list(TRANSFORM CMAKE_AUTOUIC_SEARCH_PATHS APPEND /CGAL/Qt/resources/)
message(${CMAKE_AUTOUIC_SEARCH_PATHS})
set(CGAL_QT_SOURCE
	#        ${CGAL_INCLUDE_DIRS}/CGAL/Qt/image_interface.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/qglviewer.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/domUtils.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/frame.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/manipulatedFrame.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/manipulatedCameraFrame.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/camera.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/keyFrameInterpolator.h
	#${CGAL_INCLUDE_DIRS}/CGAL/Qt/qglviewer_impl.h
	)

# External
include_directories(external)

# Own
include_directories(lib)

# IoGeometry
add_library(io_geometry STATIC lib/iogeometry.cpp lib/iogeometry.h)

# GraphStatistics
add_library(graph_statistics STATIC lib/graphStats.cpp lib/graphStats.h lib/graphStatsInline.h)

# VoxelArrangement
add_library(voxel_arrangement STATIC lib/VoxelArrangement.cpp lib/VoxelArrangement.h)

# VoxelSparse
add_library(voxel_sparse STATIC lib/VoxelSparse.cpp lib/VoxelSparse.h)

add_executable(bim_data main.cpp)

add_executable(color_classes main_color_classes.cpp)

#add_executable(closeness_checker main_closeness_checker.cpp
#        ${CGAL_QT_SOURCE} lib/meshProcessing.cpp lib/meshProcessing.h)

add_executable(plane_arrangement main_plane_arrangement.cpp)

#add_executable(boolean_operations_check
#        main_boolean_operations_check.cpp
#        ${CGAL_QT_SOURCE}
#        lib/BimObj.cpp lib/BimObj.h
#        lib/meshProcessing.cpp lib/meshProcessing.h)

add_executable(graphStatistics main_graph_statistics.cpp)

add_executable(graph2ply main_graph_to_ply.cpp )

add_executable(mesh2plane main_mesh_to_plane.cpp
        lib/RegionGrowing.cpp lib/RegionGrowing.h)

add_executable(splitGraph main_split_graph.cpp)

add_executable(trivialScanSimulator main_trivial_scan_simulator.cpp)

add_executable(edgeFeatures2ply main_edge_feature_to_ply.cpp)

add_executable(makePointEdgeFeatures main_make_point_edge_features.cpp)

add_executable(datasetPointEdgeFeatures main_dataset_point_edge_features.cpp)
target_link_libraries(datasetPointEdgeFeatures "-lstdc++fs")

add_executable(labeledPc2Obj main_labeled_pc_to_obj.cpp)

add_executable(ptViewShooter main_pt_view_shooter.cpp)

add_executable(assignLabelSingleChunk main_assign_label.cpp)

add_executable(neighbourhoodStats main_neighbourhood_stats.cpp)

add_executable(visibilityLines main_visibility_lines.cpp)

add_executable(voxelizeModel main_voxelize_model.cpp)

add_executable(splitVoxels main_split_voxels.cpp)

add_executable(voxels2ply main_voxels_to_ply.cpp)

add_executable(voxelFeatures2ply main_voxel_features_to_ply.cpp)

add_executable(pc2dense main_pc_to_dense.cpp)

add_executable(splitLabeledPoints main_split_labeled_points.cpp)

add_executable(mergePly main_merge_ply.cpp)

target_link_libraries(bim_data ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(color_classes io_geometry)
target_link_libraries(plane_arrangement io_geometry)
target_link_libraries(graph2ply io_geometry)
#target_link_libraries(closeness_checker ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL)
#target_link_libraries(boolean_operations_check ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL gmpxx io_geometry)
target_link_libraries(graphStatistics ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(splitGraph ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(trivialScanSimulator ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(edgeFeatures2ply ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(makePointEdgeFeatures ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(datasetPointEdgeFeatures ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(ptViewShooter ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(labeledPc2Obj ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(assignLabelSingleChunk ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(neighbourhoodStats ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(visibilityLines ${CGAL_LIBRARIES} io_geometry graph_statistics)
target_link_libraries(voxelizeModel ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(voxel_arrangement ${HDF5_LIBRARIES})
target_link_libraries(splitVoxels ${CGAL_LIBRARIES} io_geometry voxel_arrangement graph_statistics)
target_link_libraries(voxels2ply ${CGAL_LIBRARIES} io_geometry voxel_arrangement graph_statistics)
target_link_libraries(voxelFeatures2ply ${CGAL_LIBRARIES} io_geometry voxel_arrangement graph_statistics)
target_link_libraries(pc2dense ${CGAL_LIBRARIES} io_geometry)
target_link_libraries(splitLabeledPoints ${CGAL_LIBRARIES} io_geometry voxel_arrangement graph_statistics)
target_link_libraries(mergePly tinyply)
if(MKL_FOUND)
    target_link_libraries(mesh2plane ${MKL_LIBRARIES})
endif()

### TESTS ###

# GTest
add_library(GTest_in STATIC external/gtest/gtest.h external/gtest/gtest-all.cc)
set_target_properties(GTest_in PROPERTIES LINK_FLAGS "-pthread")

# Making tests
enable_testing()
include_directories(test)
add_executable(runUnitTests test/mainTest.cpp test/test_graph.cpp test/graph_fixture.h
        test/test_region_growing.cpp test/test_voxels.cpp lib/RegionGrowing.cpp test/test_voxels_sparse.cpp)
target_link_libraries(runUnitTests GTest_in pthread ${CGAL_LIBRARIES} io_geometry graph_statistics voxel_arrangement
        voxel_sparse)
add_test(runUnitTests runUnitTests)
