cmake_minimum_required(VERSION 3.1)
project(bim_data)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

#Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Widgets Core Xml OpenGL REQUIRED)

# CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
message("CGAL_VERSION: ${CGAL_VERSION} test")
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CGAL_INCLUDE_DIRS})
list(TRANSFORM CMAKE_AUTOUIC_SEARCH_PATHS APPEND /CGAL/Qt/resources/)
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

add_executable(bim_data
        main.cpp
        lib/iogeometry.cpp
        lib/iogeometry.h)

add_executable(color_classes
        main_color_classes.cpp
        lib/iogeometry.cpp
        lib/iogeometry.h)

add_executable(closeness_checker
        main_closeness_checker.cpp
        ${CGAL_QT_SOURCE} lib/meshProcessing.cpp lib/meshProcessing.h)

add_executable(plane_arrangement
        main_plane_arrangement.cpp
        lib/iogeometry.cpp)

add_executable(boolean_operations_check
        main_boolean_operations_check.cpp
        ${CGAL_QT_SOURCE}
        lib/iogeometry.cpp lib/iogeometry.h
        lib/BimObj.cpp lib/BimObj.h
        lib/meshProcessing.cpp lib/meshProcessing.h)

add_executable(graph_statistics main_graph_statistics.cpp
        lib/iogeometry.cpp lib/graphStats.cpp lib/graphStats.h)

add_executable(graph2ply main_graph_to_ply.cpp
        lib/iogeometry.cpp )

add_executable(mesh2plane main_mesh_to_plane.cpp
        lib/RegionGrowing.cpp lib/RegionGrowing.h)

add_executable(splitGraph main_split_graph.cpp
        lib/iogeometry.cpp lib/graphStats.cpp)

add_executable(trivialScanSimulator main_trivial_scan_simulator.cpp
        lib/iogeometry.cpp)

add_executable(makePointEdgeFeatures main_make_point_edge_features.cpp
        lib/iogeometry.cpp lib/graphStats.cpp)

add_executable(labeledPc2Obj main_labeled_pc_to_obj.cpp
        lib/iogeometry.cpp)


target_link_libraries(bim_data ${CGAL_LIBRARIES})
target_link_libraries(closeness_checker ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL)
target_link_libraries(boolean_operations_check ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL gmpxx)
target_link_libraries(graph_statistics ${CGAL_LIBRARIES})
target_link_libraries(splitGraph ${CGAL_LIBRARIES})
target_link_libraries(trivialScanSimulator ${CGAL_LIBRARIES})
target_link_libraries(makePointEdgeFeatures ${CGAL_LIBRARIES})
target_link_libraries(labeledPc2Obj ${CGAL_LIBRARIES})
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
        lib/graphStats.cpp lib/iogeometry.cpp test/test_region_growing.cpp lib/RegionGrowing.cpp)
target_link_libraries(runUnitTests GTest_in pthread ${CGAL_LIBRARIES})
add_test(runUnitTests runUnitTests)
