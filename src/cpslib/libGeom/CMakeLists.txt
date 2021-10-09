CMAKE_MINIMUM_REQUIRED(VERSION 3.19.0)

PROJECT(libGeom)

find_package(PCL 1.2 REQUIRED)

SET(CPSSPATIALAR_BASEDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
SET(EIGEN_DIR ${CPSSPATIALAR_BASEDIR}/external/eigen/)
SET(FREEGLUT_DIR ${CPSSPATIALAR_BASEDIR}/external/freeglut/)
SET(GLEW_DIR ${CPSSPATIALAR_BASEDIR}/external/glew/)
SET(GLFW_DIR ${CPSSPATIALAR_BASEDIR}/external/glfw/)
SET(OPENCV_DIR ${CPSSPATIALAR_BASEDIR}/external/opencv/)

LINK_DIRECTORIES(
	${OPENCV_DIR}/lib
	${PCL_LIBRARY_DIRS}
)

ADD_DEFINITIONS(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CPSSPATIALAR_BASEDIR}/external
	${EIGEN_DIR}
	${FREEGLUT_DIR}/include
	${GLEW_DIR}/include
	${GLFW_DIR}/include
	${OPENCV_DIR}/build
	${OPENCV_DIR}/include
	${OPENCV_DIR}/modules/calib3d/include
	${OPENCV_DIR}/modules/core/include
	${OPENCV_DIR}/modules/flann/include
	${OPENCV_DIR}/modules/features2d/include
	${OPENCV_DIR}/modules/highgui/include
	${OPENCV_DIR}/modules/imgcodecs/include
	${OPENCV_DIR}/modules/imgproc/include
	${OPENCV_DIR}/modules/videoio/include
	${OPENCV_DIR}/src/include
	${OPENCV_DIR}/src/include/opencv
	${OPENCV_DIR}/3rdparty/include
	${PCL_INCLUDE_DIRS}
)

ADD_LIBRARY(libGeom
	Camera.cpp
	Camera.h
	headers.h
	Plane.cpp
	Plane.h
	PointCloud.cpp
	PointCloud.h
)

ADD_CUSTOM_COMMAND(TARGET libGeom
	POST_BUILD
		COMMAND ${CMAKE_COMMAND}
			-DInputLibPath="${CPSSPATIALAR_BASEDIR}/build/lib/Debug/libGeomd.lib"
			-DOutputLibPath="${CPSSPATIALAR_BASEDIR}/build/lib/libGeomd.lib"
			-P "${CPSSPATIALAR_BASEDIR}/rename.cmake"
		COMMAND ${CMAKE_COMMAND}
			-DInputLibPath="${CPSSPATIALAR_BASEDIR}/build/lib/Release/libGeom.lib"
			-DOutputLibPath="${CPSSPATIALAR_BASEDIR}/build/lib/libGeom.lib"
			-P "${CPSSPATIALAR_BASEDIR}/rename.cmake"
)

SET_TARGET_PROPERTIES(libGeom
    PROPERTIES
        DEBUG_POSTFIX d
        ARCHIVE_OUTPUT_DIRECTORY "${CPSSPATIALAR_BASEDIR}/build/lib"
)