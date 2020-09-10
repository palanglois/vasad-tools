cmake_minimum_required(VERSION 3.1)
project(bim_data)

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

# EIGEN
find_package( Eigen3 3.2 REQUIRED)
include( ${EIGEN3_USE_FILE} )


# External
include_directories(external)

add_executable(bim_data
        main.cpp
        iogeometry.cpp
        iogeometry.h)

add_executable(closeness_checker
        main_closeness_checker.cpp
        ${CGAL_QT_SOURCE} meshProcessing.cpp meshProcessing.h)

add_executable(plane_arrangement
        main_plane_arrangement.cpp
        iogeometry.cpp)

add_executable(boolean_operations_check
        main_boolean_operations_check.cpp
        ${CGAL_QT_SOURCE}
        iogeometry.cpp iogeometry.h
        BimObj.cpp BimObj.h
        meshProcessing.cpp meshProcessing.h)
add_executable(main_test main_test.cpp)

target_link_libraries(bim_data ${CGAL_LIBRARIES})
target_link_libraries(closeness_checker ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL)
target_link_libraries(boolean_operations_check ${CGAL_LIBRARIES} Qt5::Widgets Qt5::Xml Qt5::OpenGL gmpxx)
target_link_libraries(main_test ${CGAL_LIBRARIES})
