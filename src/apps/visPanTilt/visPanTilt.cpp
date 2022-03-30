/*
 *   2022 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 */

// CMake projects should use: "-DCMAKE_TOOLCHAIN_FILE=C:/Users/micha/vcpkg/scripts/buildsystems/vcpkg.cmake"

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>

#include "headers.h"
#include "Camera.h"
#include "PointCloud.h"
#include "Plane.h"
#include "CalibPattern.h"
#include "Homography.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

float M_PI = 3.14159265358979323846;

using namespace std;
using namespace cv;


GLint viewportWidth;
GLint viewportHeight;

GLFWwindow * window;

bool showDistances;
bool showVirtual;
bool showImVis;

/* Camera variables */
GLdouble defaultCameraDist, defaultCameraYaw, defaultCameraRoll;
GLdouble cameraDist, cameraYaw, cameraRoll;
GLdouble * cameraPos;

int camIdx;
int viewIdx;

std::vector<Camera> cams;
Camera projCanonical;

PointCloud pointCloud;

int imWidth, imHeight;

/* Previous x and y cursor coordinates (used to determine change in camera position) */
double cursorPrevX, cursorPrevY;

/* Previous mouse wheel position (used to determine zoom factor) */
int resizePointsMousePrevWheelPos;
int mousePrevWheelPos;
int timeMousePrevWheelPos;
int timeBetZeroAndOne;

std::vector<std::vector<float>> panTilt;

float x, y, cloudX, cloudY, panOffset, tiltOffset;
float initX, initY, initCloudX, initCloudY;
float step;
int arrowKeyStep;
float cloudArrowKeyStep;
bool lockMouse;
bool fastMove;
int fastMult;
float pointSize;
float defaultPointSize;
bool hasAlt;

bool newEvent;
bool newArrowEvent;

float radius;

GLfloat * lightPos;

bool showMesh;
bool toggleWireframe;
bool resizePoints;

static const char* keys =
{
    "{@boardSqSize | | ...}"
    "{@boardDimsX | | ...}"
    "{@boardDimsY | | ...}"
    "{@circlesDimsX | | ...}"
    "{@circlesDimsY | | ...}"
    "{@outDir | | ...}"
    "{@numIms | | ...}"
    "{@cam0ImDir | | ...}"
    "{@cam1ImDir | | ...}"
    "{@cam0Path | | ...}"
    "{@cam1Path | | ...}"
    "{@circlesImPath | | ...}"
    "{@targetWidth | | ...}"
    "{@visImIdx | | ...}"
    "{@visIm | | ...}"
};

void help()
{
    cout << "visPanTilt <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <cam0ImDir> <cam1ImDir> <cam0Path> <cam1Path> <circlesImPath> <targetWidth> [<visImIdx>] [<visIm>]\n"
        << endl;
}

void reflectProj(Camera & proj, float pan, float tilt, cv::Mat & outProjReflectedPose34d, Plane & outPlane)
{
    // we'll optimize over these
    float delta_p = 86 / 1000.0; // length of PAN axis
    float delta_t = 24 / 1000.0; // offset of mirror from the PAN axis
    float delta_0 = 200 / 1000.0; // offset of mirror system origin relative to projector

    // rotation angles in degrees
    float rot_p = pan + panOffset; // -90;
    float rot_t = tilt + tiltOffset; // 0;

    // rotation angles in radians
    float w_p = rot_p * M_PI / 180.0;
    float w_t = rot_t * M_PI / 180.0;

    Eigen::Vector3d M0(0, 0, delta_0); // mirror system origin relative to projector
    Eigen::Vector3d dt(0, delta_t, 0); // offset vector
    Eigen::Vector3d dp(0, 0, -delta_p); // offset vector

    Eigen::Matrix3d R_t; // rotation matrix for Tilt(X axis)
    R_t << 1, 0, 0,
        0, std::cos(w_t), -std::sin(w_t),
        0, std::sin(w_t), std::cos(w_t);

    Eigen::Matrix3d R_p; // rotation matrix for Pan(Z axis)
    R_p << std::cos(w_p), -std::sin(w_p), 0,
        std::sin(w_p), std::cos(w_p), 0,
        0, 0, 1;

    Eigen::Vector3d MP = M0 + dp;
    Eigen::Vector3d MC = M0 + R_p * (R_t * dt) + dp; // Mirror "center" MC
    Eigen::Vector3d mn = (MC - MP) / std::sqrt((MC - MP).dot(MC - MP)); // normalized mn mirror plane normal

    outPlane = Plane(cv::Vec3d(mn[0], mn[1], mn[2]), cv::Vec3d(MC[0], MC[1], MC[2]));
    cv::Vec3d lookDirPlaneIntersection = outPlane.intersect(proj.getLookDir());

    Eigen::Matrix4d translationMat;
    translationMat << 1, 0, 0, -lookDirPlaneIntersection[0],
        0, 1, 0, -lookDirPlaneIntersection[1],
        0, 0, 1, -lookDirPlaneIntersection[2],
        0, 0, 0, 1;

    Eigen::Matrix4d translationMatInv = translationMat.inverse();

    Eigen::Matrix4d projPose;
    cv::Mat projPoseCV = proj.getRt44();
    cv::Mat projPoseInvCV = proj.getRt44Inv();
    cv::cv2eigen(projPoseCV, projPose);

    cv::Vec3d planeXCV = outPlane.intersect(cv::Vec3d(
        projPoseCV.at<double>(0, 0),
        projPoseCV.at<double>(0, 1),
        projPoseCV.at<double>(0, 2))) -
        outPlane.intersect(cv::Vec3d(
            projPoseCV.at<double>(2, 0),
            projPoseCV.at<double>(2, 1),
            projPoseCV.at<double>(2, 2)));

    // handle case where X axis is parallel to plane
    if (std::isnan(planeXCV[2]))
    {
        planeXCV = cv::Vec3d(
            projPoseCV.at<double>(0, 0),
            projPoseCV.at<double>(0, 1),
            projPoseCV.at<double>(0, 2));
    }

    planeXCV /= std::sqrt(planeXCV.dot(planeXCV));

    Eigen::Vector3d planeZ = mn;
    Eigen::Vector3d planeX(planeXCV[0], planeXCV[1], planeXCV[2]);
    Eigen::Vector3d planeY = planeX.cross(planeZ);

    Eigen::Matrix4d rotMatInv;
    rotMatInv << planeX[0], planeY[0], planeZ[0], 0,
        planeX[1], planeY[1], planeZ[1], 0,
        planeX[2], planeY[2], planeZ[2], 0,
        0, 0, 0, 1;

    Eigen::Matrix4d rotMat = rotMatInv.inverse();

    Eigen::Matrix4d reflectionMat;
    reflectionMat << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4d transformationMat = translationMatInv * rotMatInv * reflectionMat * rotMat * translationMat;
    Eigen::Matrix4d projVirtualPose = transformationMat * projPose;

    cv::Mat projVirtualPoseCV;
    cv::eigen2cv(projVirtualPose, projVirtualPoseCV);

    cv::Mat Rt34d(3, 4, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            Rt34d.at<double>(i, j) = projVirtualPoseCV.at<double>(i, j);

    Rt34d.copyTo(outProjReflectedPose34d);
    outPlane.rigidTransform(projPoseInvCV);
}

void displayText(float x, float y, float z, float r, float g, float b, const char* string)
{
    glColor3f(r, g, b);
    glRasterPos3f(x, y, z);

    while (*string) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *string++);
    }
}

void displayText(float x, float y, float r, float g, float b, const char* string)
{
    glColor3f(r, g, b);
    glWindowPos2i(x, y);

    while (*string) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *string++);
    }
}

void display()
{
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    if (newEvent)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glTranslated(0, 0, -cameraDist);
        glTranslated(-cloudX, cloudY, 0);

        glRotated(cameraYaw, 0.0, 1.0, 0.0); /* 'Horizontal' rotation based on change in cursor's y location */
        glRotated(cameraRoll, 1.0, 0.0, 0.0); /* 'Vertical' rotation based on change in cursor's x location */

        double flip[] = { 1,  0,  0,  0,
                          0, -1,  0,  0,
                          0,  0, -1,  0,
                          0,  0,  0,  1 };

        glMultMatrixd(flip);

        glMultMatrixd(Ancillary::flattenMat44d(projCanonical.getRt44()));

        projCanonical.displayWorld(0.0, 0.0, 0.0);

        std::stringstream ss;
        ss << "panOffset " << panOffset << "; tiltOffset " << tiltOffset;

        for (int numIm = 0; numIm < 1 /*panTilt.size()*/; numIm++)
        {
            float pan = panTilt[numIm][0];
            float tilt = panTilt[numIm][1];

            cv::Mat projReflectedPose34d;
            Plane plane;
            reflectProj(projCanonical, pan, tilt, projReflectedPose34d, plane);

            // projReflected.displayWorld(0.5, 0.5, 0.5);
            Camera(projCanonical.getK(), projReflectedPose34d, projCanonical.getWidth(), projCanonical.getHeight(), 0.005).displayWorld(0.5, 0.5, 0.5);
            plane.display(0.1);

            std::cout << "projReflectedPose34d: " << projReflectedPose34d << std::endl;

            //glColor3f(0, 1, 0); // green

            //glBegin(GL_LINES);
            //    glVertex3f(0, 0, 0);
            //    glVertex3f(M0[0], M0[1], M0[2]);
            //glEnd();

            //glColor3f(0, 0, 1); // blue 

            //glBegin(GL_LINES);
            //    glVertex3f(M0[0], M0[1], M0[2]);
            //    glVertex3f(MP[0], MP[1], MP[2]);
            //glEnd();

            //glColor3f(0, 1, 1); // light blue

            //glBegin(GL_LINES);
            //    glVertex3f(MP[0], MP[1], MP[2]);
            //    glVertex3f(MC[0], MC[1], MC[2]);
            //glEnd();
        }

        //// here we compute pose of virtual projector relative to canonical projector
        //{
        //    // rotation in degrees
        //    float rot_p = pan - 90;
        //    float rot_t = tilt;

        //    // this never ever changes
        //    // all in mm = > 1px = 1mm
        //    float delta_p = 86 / 1000.0;                 // length of PAN axis
        //    float delta_t = 24 / 1000.0;                // offset of mirror from the PAN axis

        //    Eigen::Vector3d PC(projCanonical.getC()[0], projCanonical.getC()[1], projCanonical.getC()[2]); // position of projector

        //    cv::Vec3d lookDirCV = projCanonical.backprojectLocal(projCanonical.getPrincipalPt());
        //    Eigen::Vector3d Pf(lookDirCV[0], lookDirCV[1], lookDirCV[2]); // look dir of projector
        //    //Eigen::Vector3d Pf(projCanonical.getLookDir()[0], projCanonical.getLookDir()[1], projCanonical.getLookDir()[2]); // look dir of projector

        //    Eigen::Vector3d M0(0, 0, 200 / 1000.0); // mirror system origin relative to canonical pose of projector
        //    Eigen::Vector3d dt(0, delta_t, 0); // offset vector
        //    Eigen::Vector3d dp(0, 0, -delta_p); // offset vector

        //    float w_p = rot_p * M_PI / 180.0;
        //    float w_t = rot_t * M_PI / 180.0;

        //    Eigen::Matrix3d R_t; // rotation matrix for Tilt(X axis)
        //    R_t << 1, 0, 0,
        //        0, std::cos(w_t), -std::sin(w_t),
        //        0, std::sin(w_t), std::cos(w_t); 

        //    Eigen::Matrix3d R_p; // rotation matrix for Pan(Z axis)
        //    R_p << std::cos(w_p), -std::sin(w_p), 0,
        //        std::sin(w_p), std::cos(w_p), 0,
        //        0, 0, 1;

        //    Eigen::Vector3d MP = M0 + dp;
        //    Eigen::Vector3d MC = M0 + R_p * (R_t * dt) + dp; // Mirror "center" MC
        //    Eigen::Vector3d mn = (MC - MP) / sqrt((MC - MP).dot(MC - MP)); // normalized mn mirror plane normal

        //    Eigen::Vector3d PC_dash = PC - 2 * (PC - MC).dot(mn) * mn;  // new projector center
        //    Eigen::Vector3d Pf_dash = Pf - 2 * Pf.dot(mn) * mn; // new normalized forward projection vector

        //    std::cout << Pf_dash << std::endl;

        //    //glBegin(GL_POINTS);
        //    //glVertex3f(PC_dash[0], PC_dash[1], PC_dash[2]);
        //    //glEnd();

        //    //glBegin(GL_LINES);
        //    //    glVertex3f(PC_dash[0], PC_dash[1], PC_dash[2]);
        //    //    glVertex3f(PC_dash[0] + Pf_dash[0], PC_dash[1] + Pf_dash[1], PC_dash[2] + Pf_dash[2]);
        //    //glEnd();

        //    std::cout << "dist(M0, MP) = " << sqrt((M0 - MP).dot(M0 - MP)) << std::endl;
        //    std::cout << "dist(MP, MC) = " << sqrt((MP - MC).dot(MP - MC)) << std::endl;

        //    Plane plane(cv::Vec3d(mn[0], mn[1], mn[2]), cv::Vec3d(MC[0], MC[1], MC[2]));
        //    plane.display(0.1);

        //    cv::Vec3d lookDirPlaneIntersection = plane.intersect(projCanonical.getLookDir());

        //    Eigen::Matrix4d translationMat;
        //    translationMat << 1, 0, 0, -lookDirPlaneIntersection[0],
        //        0, 1, 0, -lookDirPlaneIntersection[1],
        //        0, 0, 1, -lookDirPlaneIntersection[2],
        //        0, 0, 0, 1;

        //    Eigen::Matrix4d translationMatInv = translationMat.inverse();

        //    Eigen::Matrix4d projCanonicalPose;
        //    cv::Mat projCanonicalPoseCV = projCanonical.getRt44();
        //    cv::cv2eigen(projCanonicalPoseCV, projCanonicalPose);

        //    cv::Vec3d planeXCV = plane.intersect(cv::Vec3d(
        //        projCanonicalPoseCV.at<double>(0, 0),
        //        projCanonicalPoseCV.at<double>(0, 1),
        //        projCanonicalPoseCV.at<double>(0, 2))) -
        //        plane.intersect(cv::Vec3d(
        //            projCanonicalPoseCV.at<double>(2, 0),
        //            projCanonicalPoseCV.at<double>(2, 1),
        //            projCanonicalPoseCV.at<double>(2, 2)));

        //    if (std::isnan(planeXCV[2]))
        //    {
        //        planeXCV = cv::Vec3d(
        //            projCanonicalPoseCV.at<double>(0, 0),
        //            projCanonicalPoseCV.at<double>(0, 1),
        //            projCanonicalPoseCV.at<double>(0, 2));
        //    }

        //    planeXCV /= std::sqrt(planeXCV.dot(planeXCV));

        //    Eigen::Vector3d planeZ = mn;
        //    Eigen::Vector3d planeX(planeXCV[0], planeXCV[1], planeXCV[2]);
        //    Eigen::Vector3d planeY = planeX.cross(planeZ);


        //    Eigen::Matrix4d rotMatInv;
        //    rotMatInv << planeX[0], planeY[0], planeZ[0], 0,
        //        planeX[1], planeY[1], planeZ[1], 0,
        //        planeX[2], planeY[2], planeZ[2], 0,
        //        0, 0, 0, 1;

        //    std::cout << "rotMatInv: " << rotMatInv << std::endl;

        //    Eigen::Matrix4d rotMat = rotMatInv.inverse();

        //    std::cout << "rotMat: " << rotMat << std::endl;

        //    Eigen::Matrix4d reflectionMat;
        //    reflectionMat << 1, 0, 0, 0,
        //        0, 1, 0, 0,
        //        0, 0, -1, 0,
        //        0, 0, 0, 1;

        //    
        //    Eigen::Matrix4d transformationMat = translationMatInv * rotMatInv * reflectionMat * rotMat * translationMat;
        //    Eigen::Matrix4d projVirtualPose = transformationMat * projCanonicalPose;
        //    std::cout << "transformationMat: " << transformationMat << std::endl;
        //    std::cout << "projCanonicalPose: " << projCanonicalPose << std::endl;
        //    std::cout << "projVirtualPose: " << projVirtualPose << std::endl;

        //    cv::Mat projVirtualPoseCV;
        //    cv::eigen2cv(projVirtualPose, projVirtualPoseCV);
        //    
        //    cv::Mat Rt34d(3, 4, CV_64F);
        //    for (int i = 0; i < 3; i++)
        //        for (int j = 0; j < 4; j++)
        //            Rt34d.at<double>(i, j) = projVirtualPoseCV.at<double>(i, j);

        //    Camera projVirtual(projCanonical.getK(), Rt34d, projCanonical.getWidth(), projCanonical.getHeight(), 0.005);

        //    projVirtual.displayWorld(0.5, 0.5, 0.5);

        //    glBegin(GL_LINES);
        //        glVertex3f(PC[0], PC[1], PC[2]);
        //        glVertex3f(lookDirPlaneIntersection[0], lookDirPlaneIntersection[1], lookDirPlaneIntersection[2]);
        //    glEnd();

        //    glBegin(GL_LINES);
        //        glVertex3f(projVirtual.getC()[0], projVirtual.getC()[1], projVirtual.getC()[2]);
        //        glVertex3f(lookDirPlaneIntersection[0], lookDirPlaneIntersection[1], lookDirPlaneIntersection[2]);
        //    glEnd();

        //    glColor3f(0, 1, 0); // green

        //    glBegin(GL_LINES);
        //        glVertex3f(0, 0, 0);
        //        glVertex3f(M0[0], M0[1], M0[2]);
        //    glEnd();

        //    glColor3f(0, 0, 1); // blue 

        //    glBegin(GL_LINES);
        //    glVertex3f(M0[0], M0[1], M0[2]);
        //    glVertex3f(MP[0], MP[1], MP[2]);
        //    glEnd();

        //    glColor3f(0, 1, 1); // light blue

        //    glBegin(GL_LINES);
        //    glVertex3f(MP[0], MP[1], MP[2]);
        //    glVertex3f(MC[0], MC[1], MC[2]);
        //    glEnd();

        //    //glColor3f(1, 0, 1);
        //    //glBegin(GL_LINES);
        //    //    glVertex3f(0, 0, 0);
        //    //    glVertex3f(1, 0, 0);
        //    //glEnd();
        //    //glBegin(GL_LINES);
        //    //    glVertex3f(0, 0, 0);
        //    //    glVertex3f(0, 1, 0);
        //    //glEnd();
        //    //glBegin(GL_LINES);
        //    //    glVertex3f(0, 0, 0);
        //    //    glVertex3f(0, 0, 1);
        //    //glEnd();
        //}

        displayText(15, 20, 0, 0, 0, ss.str().c_str());

        glFlush();
        glfwSwapBuffers(window);

        newEvent = false;
    }
}

void reshape(int width, int height)
{
    if (width == -1)
        width = glutGet(GLUT_SCREEN_WIDTH);

    if (height == -1)
        height = glutGet(GLUT_SCREEN_HEIGHT);

    float scaleFactor = 1.0;

	float w = projCanonical.getWidth();
    float h = projCanonical.getHeight();

    if (w > width || h > height)
        scaleFactor = 0.5;

    w *= scaleFactor;
    h *= scaleFactor;

    viewportWidth = (GLint)w;
	viewportHeight = (GLint)h;

	glfwSetWindowSize(window, viewportWidth, viewportHeight);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, viewportWidth, viewportHeight);
    glMatrixMode( GL_PROJECTION );

    float farPlane = 20000;
    float nearPlane = 0.0;

    double f = projCanonical.getf() * scaleFactor;
    double cx = projCanonical.getPrincipalPt()[0] * scaleFactor;
    double cy = projCanonical.getPrincipalPt()[1] * scaleFactor;

	// http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl
    // todo: compare with http://cvrr.ucsd.edu/publications/2008/MurphyChutorian_Trivedi_CVGPU08.pdf
    double matTransp[] = { 2*f/w,     0,         0,                                          0,
                           0,         2*f/h,     0,                                          0,
                          1-2*cx/w,  -1 + (2*cy + 2)/h, (farPlane+nearPlane)/(nearPlane - farPlane), -1,
                           0,         0,        2*farPlane*nearPlane/(nearPlane-farPlane),  0};
	
    glLoadMatrixd(matTransp);
	
	newEvent = true;
	display();
}

void reshape(GLFWwindow* window, int width, int height)
{
    reshape(width, height);
}

void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key)
    {
    case GLFW_KEY_F1:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            cameraYaw = 0;
            cameraRoll = 0;
            cameraDist = 0;

            cameraRoll = defaultCameraRoll;
            cameraYaw = defaultCameraYaw;

            cloudX = initCloudX;
            cloudY = initCloudY;

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_SPACE:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            lockMouse = (lockMouse) ? false : true;
        }
        break;
    case GLFW_KEY_LEFT_SHIFT:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            fastMove = true;
        else
            fastMove = false;
        break;
    case GLFW_KEY_RIGHT_SHIFT:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            fastMove = true;
        else
            fastMove = false;
        break;
    case GLFW_KEY_LEFT_CONTROL:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            resizePoints = true;
        else
            resizePoints = false;
        break;
    case GLFW_KEY_RIGHT_CONTROL:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            resizePoints = true;
        else
            resizePoints = false;
        break;
    case GLFW_KEY_LEFT_ALT:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            hasAlt = true;
        else
            hasAlt = false;
        break;
    case GLFW_KEY_RIGHT_ALT:
        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            hasAlt = true;
        else
            hasAlt = false;
        break;
    case GLFW_KEY_ESCAPE:
        glfwDestroyWindow(window);
        exit(0);
    default:
        break;
    }
}

void camera(GLFWwindow* window, double x, double y)
{
    if (!lockMouse)
    {
        /* Change angle (in degrees) of 'horizontal' camera rotation w.r.t. origin */
        cameraYaw += 0.5 * (x - cursorPrevX);

        /* Change angle (in degrees) of 'vertical' camera rotation w.r.t. origin */
        GLdouble deltaCameraRoll = cameraRoll + 0.5 * (y - cursorPrevY);
        cameraRoll = deltaCameraRoll;

        cursorPrevX = x;
        cursorPrevY = y;

        newEvent = true;
    }
}

void zoom(GLFWwindow* window, double xoffset, double pos)
{
    GLdouble tempCameraDist;
    GLdouble tempPointSize;
    float tempTimeBetZeroAndOne;

    if (resizePoints)
    {
        if (pos == 1)
            tempPointSize = pointSize + 1;
        else if (pos == -1)
            tempPointSize = pointSize - 1;
        else
            tempPointSize = pointSize;

        if (tempPointSize > 0.5)
            pointSize = tempPointSize;

        resizePointsMousePrevWheelPos = pos;
    }
    else if (hasAlt)
    {
        if (pos == 1)
        {
            tempTimeBetZeroAndOne = timeBetZeroAndOne + ((fastMove) ? 0.05 : 0.01);
            if (tempTimeBetZeroAndOne < 0) tempTimeBetZeroAndOne = 0;
        }
        else if (pos == -1)
        {
            tempTimeBetZeroAndOne = timeBetZeroAndOne - ((fastMove) ? 0.05 : 0.01);
            if (tempTimeBetZeroAndOne > 1) tempTimeBetZeroAndOne = 1;
        }

        timeBetZeroAndOne = tempTimeBetZeroAndOne;

        timeMousePrevWheelPos = pos;
    }
    else
    {
        if (pos == 1)
            tempCameraDist = cameraDist + ((fastMove) ? fastMult * step : step);
        else if (pos == -1)
            tempCameraDist = cameraDist - ((fastMove) ? fastMult * step : step);
        else
            tempCameraDist = cameraDist;

        cameraDist = tempCameraDist;

        mousePrevWheelPos = pos;
    }

    newEvent = true;
}

void init(double w, double h)
{
    panOffset = -90; //90;
    tiltOffset = 0; //45;
    
    viewportWidth = w;
    viewportHeight = h;

    showDistances = false;
    showVirtual = false;
    showImVis = false;

    cursorPrevX = 0;
    cursorPrevY = 0;

    step = 0.5;
    arrowKeyStep = 0.001;
    cloudArrowKeyStep = 0.001;
    lockMouse = false;
    fastMove = false;
    fastMult = 10;
    toggleWireframe = false;
    defaultPointSize = 10;
    pointSize = defaultPointSize;
    resizePoints = false;
    hasAlt = false;
    mousePrevWheelPos = 0;
    timeMousePrevWheelPos = 0;
    timeBetZeroAndOne = 0;
    radius = 50;

    camIdx = 0;
    viewIdx = 0;

    initCloudX = 0;
    initCloudY = 0;

    newEvent = true;
    newArrowEvent = true;

    showMesh = false;

    /* Initialize camera variables */
    defaultCameraDist = 0;
    defaultCameraRoll = 0;
    defaultCameraYaw = 0;

    cameraDist = defaultCameraDist;
    cameraRoll = defaultCameraRoll;
    cameraYaw = defaultCameraYaw;

    glfwInit();

    window = glfwCreateWindow(viewportWidth, viewportHeight, "visPanTilt (IMW-CPS TU Vienna / michael.hornacek@gmail.com)", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create window" << std::endl;
        exit(-1);
    }

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    glewInit();

    glfwGetCursorPos(window, &cursorPrevX, &cursorPrevY);

    glfwSetWindowSizeCallback(window, reshape);
    glfwSetCursorPosCallback(window, camera);
    glfwSetScrollCallback(window, zoom);
    glfwSetKeyCallback(window, keyboard);

    glClearDepth(1.0f);

    glShadeModel(GL_SMOOTH);
    //glShadeModel(GL_FLAT);
    //glEnable(GL_COLOR_MATERIAL);

    /* Set material parameters */
    GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat shininess[] = { 100.0 };
    //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    /* Set lighting parameters */

    cv::Vec3d pos(0, 0, -1);
    cv::Mat rot(3, 3, CV_64F);

    float angle = -PI / 5.0;
    rot.at<double>(0, 0) = 1;
    rot.at<double>(1, 0) = 0;
    rot.at<double>(2, 0) = 0;
    rot.at<double>(0, 1) = 0;
    rot.at<double>(1, 1) = std::cos(angle);
    rot.at<double>(2, 1) = std::sin(angle);
    rot.at<double>(0, 2) = 0;
    rot.at<double>(1, 2) = -std::sin(angle);
    rot.at<double>(2, 2) = std::cos(angle);

    pos = Ancillary::Mat33dTimesVec3d(rot, pos);

    lightPos = new GLfloat[4];
    lightPos[0] = pos[0];
    lightPos[1] = pos[1];
    lightPos[2] = pos[2];
    lightPos[3] = 0.0;
    GLfloat emissive[] = { 0.75, 0.75, 0.75, 1.0 };
    GLfloat diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat ambient[] = { 1, 1, 1, 1.0 };
    //glLightfv(GL_LIGHT0, GL_EMISSION, emissive);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    //glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    /* Some optimizations... */
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    /* Keep normals at length 0 in spite of zooming */
    glEnable(GL_RESCALE_NORMAL);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

    glPointSize(defaultPointSize);
    glLineWidth(1.0);
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 14)
    {
        help();
        return -1;
    }

    float chessboardSqSize = parser.get<float>(0);
    int chessboardDimsX = parser.get<int>(1);
    int chessboardDimsY = parser.get<int>(2);
    int circlesDimsX = parser.get<int>(3);
    int circlesDimsY = parser.get<int>(4);
    String outDir = parser.get<String>(5);
    int numIms = parser.get<int>(6);
    String cam0ImDir = parser.get<String>(7);
    String cam1ImDir = parser.get<String>(8);
    String cam0Path = parser.get<String>(9);
    String cam1Path = parser.get<String>(10);
    String circlesImPath = parser.get<String>(11);
    float targetWidth = parser.get<float>(12);
    String anglesCSVPath = parser.get<String>(13);

    {
        std::vector<std::string> lines;

        std::ifstream input_file(anglesCSVPath);
        if (!input_file.is_open()) {
            std::cerr << "Could not open the file - '"
                << anglesCSVPath << "'" << endl;
            return EXIT_FAILURE;
        }

        std::string line;
        while (std::getline(input_file, line)) {

            std::vector<float> tokens;
            std::stringstream check1(line);
            std::string intermediate;

            while (std::getline(check1, intermediate, ','))
                tokens.push_back(std::stof(intermediate));

            panTilt.push_back(tokens);
        }

        for (int i = 0; i < panTilt.size(); i++)
        {
            for (int j = 0; j < panTilt[i].size(); j++)
            {
                std::cout << panTilt[i][j] << " ";
            }
            std::cout << "\n";
        }

        input_file.close();
    }

    int visImIdx = 0;
    if (argc >= 16)
        visImIdx = parser.get<int>(14);

    string visImPath;
    bool hasVisIm = false;
    if (argc == 17)
    {
        visImPath = parser.get<String>(14);
        hasVisIm = true;
    }


    // read in intrinsics and extrinsics of cam0
    FileStorage fsCam0;
    fsCam0.open(cam1Path, FileStorage::READ);

    cv::Mat cam0K, cam0R, cam0T, cam0DistCoeffs;
    fsCam0["K"] >> cam0K;
    fsCam0["R"] >> cam0R;
    fsCam0["t"] >> cam0T;
    fsCam0["distCoeffs"] >> cam0DistCoeffs;

    int imWidth, imHeight;
    fsCam0["width"] >> imWidth;
    fsCam0["height"] >> imHeight;
    cv::Size camSize(imWidth, imHeight);

    init(imWidth, imHeight);

    cams.push_back(Camera(
        cam0K, cam0R, cam0T,
        imWidth, imHeight, 0.005));

    projCanonical = Camera(cams[0]);

    glutInit(&argc, argv);
    reshape(-1, -1);

    /* Main event loop */
    while (true)
    {
        glfwPollEvents();

        /* Navigation handling (I could not figure out how to handle key combinations using keyboard callback) */
        if ((glfwGetKey(window, 'W')) && (glfwGetKey(window, 'D'))) // NE
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, 'S')) && (glfwGetKey(window, 'D'))) // SE
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, 'S')) && (glfwGetKey(window, 'A'))) // SW
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, 'W')) && (glfwGetKey(window, 'A'))) // NW
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, 'W')) // N
        {
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, 'D'))	// E
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, 'S')) // S
        {
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, 'A')) // W
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }

        // pan (up, down), tilt (left, right)
        else if (glfwGetKey(window, GLFW_KEY_UP)) // N
        {
            panOffset -= (fastMove) ? fastMult * step : step;

            if (panOffset < -180)
                panOffset = -180;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_RIGHT))	// E
        {
            tiltOffset += (fastMove) ? fastMult * step : step;

            if (tiltOffset > 90)
                tiltOffset = 90;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_DOWN)) // S
        {
            panOffset += (fastMove) ? fastMult * step : step;

            if (panOffset > 180)
                panOffset = 180;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_LEFT)) // W
        {
            tiltOffset -= (fastMove) ? fastMult * step : step;

            if (tiltOffset < -90)
                tiltOffset = -90;

            newEvent = true;
        }

        /* Render the scene */
        display();
    }
}
