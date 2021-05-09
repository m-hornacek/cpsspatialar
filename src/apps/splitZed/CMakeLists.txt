CMAKE_MINIMUM_REQUIRED(VERSION 3.19.0)

PROJECT(splitZed)
 
SET(CPSSPATIALAR_BASEDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
SET(CPSLIB_DIR ${CPSSPATIALAR_BASEDIR}/cpslib)
SET(EIGEN_DIR ${CPSSPATIALAR_BASEDIR}/external/eigen/)
SET(FREEGLUT_DIR ${CPSSPATIALAR_BASEDIR}/external/freeglut/)
SET(GLEW_DIR ${CPSSPATIALAR_BASEDIR}/external/glew/)
SET(GLFW_DIR ${CPSSPATIALAR_BASEDIR}/external/glfw/)
SET(OPENCV_DIR ${CPSSPATIALAR_BASEDIR}/external/opencv/)

LINK_DIRECTORIES(
	${CPSSPATIALAR_BASEDIR}/build/cpslib
	${FREEGLUT_DIR}/lib/x64
	${GLEW_DIR}/lib/Release
	${GLFW_DIR}/build/src/Release
	${OPENCV_DIR}/build/lib/Release
)

INCLUDE_DIRECTORIES(
	${CPSSPATIALAR_BASEDIR}/cpslib/libgeom
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
)

ADD_EXECUTABLE(splitZed
	splitZed.cpp
)

TARGET_LINK_LIBRARIES(splitZed
	optimized libGeom
	optimized glfw3
	optimized freeglut
	optimized glew32
	optimized opencv_calib3d451
	optimized opencv_core451
	optimized opencv_flann451
	optimized opencv_features2d451
	optimized opencv_highgui451
	optimized opencv_imgproc451
	optimized opencv_imgcodecs451
	optimized opencv_imgproc451
	optimized opencv_videoio451
)