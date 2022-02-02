CMAKE_MINIMUM_REQUIRED(VERSION 3.19.0)

PROJECT(visPanTilt)

find_package(PCL 1.2 REQUIRED)
find_package(Ceres REQUIRED)
 
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
	${PCL_LIBRARY_DIRS}
)

ADD_DEFINITIONS(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(
	${CPSSPATIALAR_BASEDIR}/cpslib/libCalib
	${CPSSPATIALAR_BASEDIR}/cpslib/libGeom
	${CPSSPATIALAR_BASEDIR}/cpslib/libHomography
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

ADD_EXECUTABLE(visPanTilt
	visPanTilt.cpp
)

TARGET_LINK_LIBRARIES(visPanTilt
	optimized libCalib
	optimized libGeom
	optimized libHomography
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
	${PCL_LIBRARIES}
	Ceres::ceres
)