CMAKE_MINIMUM_REQUIRED(VERSION 3.19.0)

PROJECT(cpsspatialar)

SET(CMAKE_BUILD_TYPE Release)
SET(CPSSPATIALAR_BASEDIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

SET(LIBRARY_OUTPUT_PATH "${CPSSPATIALAR_BASEDIR}/build/lib" CACHE PATH "Output dir for cpslib")
SET(EXECUTABLE_OUTPUT_PATH "${CPSSPATIALAR_BASEDIR}/build/bin" CACHE PATH "Output dir for apps")

# cpslib
ADD_SUBDIRECTORY(cpslib/libCalib)
ADD_SUBDIRECTORY(cpslib/libGeom)
ADD_SUBDIRECTORY(cpslib/libHomography)

# apps
ADD_SUBDIRECTORY(apps/applyHomography)
ADD_SUBDIRECTORY(apps/rectify)
ADD_SUBDIRECTORY(apps/undistort)
ADD_SUBDIRECTORY(apps/calibrateCam)
ADD_SUBDIRECTORY(apps/calibrateProj)
ADD_SUBDIRECTORY(apps/calibrateProjMirror)
ADD_SUBDIRECTORY(apps/visPanTilt)
ADD_SUBDIRECTORY(apps/splitZed)

FILE(COPY
	# OpenCV
	"${CPSSPATIALAR_BASEDIR}/src/external/opencv/build/bin/Release/opencv_calib3d451.dll"
	"${CPSSPATIALAR_BASEDIR}/src/external/opencv/build/bin/Release/opencv_core451.dll"
	"${CPSSPATIALAR_BASEDIR}/src/external/opencv/build/bin/Release/opencv_features2d451.dll"
	"${CPSSPATIALAR_BASEDIR}/src/external/opencv/build/bin/Release/opencv_highgui451.dll"
	"${CPSSPATIALAR_BASEDIR}/src/external/opencv/build/bin/Release/opencv_imgproc451.dll"

	# freeglut
	"${CPSSPATIALAR_BASEDIR}/src/external/freeglut/bin/x64/freeglut.dll"
	
	# glew
	"${CPSSPATIALAR_BASEDIR}/src/external/glew/bin/Release/glew32.dll"

	DESTINATION "${CPSSPATIALAR_BASEDIR}/build/bin/Release"
)

