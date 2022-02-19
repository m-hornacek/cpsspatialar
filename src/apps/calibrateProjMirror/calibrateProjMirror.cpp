/*
 *   2022 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   calibrateProj <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <cam0ImDir> <cam1ImDir> <cam0Path> <cam1Path> <circlesImPath> <targetWidth> [<visImIdx>] [<visIm>]
 *
 *   Example invocation:
 *   calibrateProjMirror 0.0565 4 6 4 11 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out 11 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outRight C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml  C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_1.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\acircles_pattern_960x600.png 1.0 3 C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png
 */

// CMake projects should use: "-DCMAKE_TOOLCHAIN_FILE=C:/Users/micha/vcpkg/scripts/buildsystems/vcpkg.cmake"

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

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
vector<Camera> cams;
int imWidth, imHeight;

Camera projCam;
int projWidth, projHeight;

int numPatterns;
vector<vector<Mat>> capturedPattern;

std::vector<PointCloud*> pointCloudCirclesVec;
PointCloud* pointCloud2Circles;
std::vector<PointCloud*> triangulatedPointCloudCirclesVec;
PointCloud* pointCloudImVis;
PointCloud* pointCloud2ImVis;

Plane* fittedPlane;

cv::Vec3d projCamIntersection;
cv::Vec3d projPlaneIntersection;
cv::Vec3d camPlaneIntersection;
cv::Vec3d virtualCamPlaneIntersection;

/* Previous x and y cursor coordinates (used to determine change in camera position) */
double cursorPrevX, cursorPrevY;

/* Previous mouse wheel position (used to determine zoom factor) */
int resizePointsMousePrevWheelPos;
int mousePrevWheelPos;
int timeMousePrevWheelPos;
int timeBetZeroAndOne;

float x, y, cloudX, cloudY;
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
    cout << "calibrate <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <cam0ImDir> <cam1ImDir> <cam0Path> <cam1Path> <circlesImPath> <targetWidth> [<visImIdx>] [<visIm>]\n"
        << endl;
}

void reflectProj(Camera& proj, float pan, float tilt, cv::Mat& outProjReflectedPose34d, Plane& outPlane)
{
    // we'll optimize over these
    float delta_p = 86 / 1000.0; // length of PAN axis
    float delta_t = 24 / 1000.0; // offset of mirror from the PAN axis
    float delta_0 = 200 / 1000.0; // offset of mirror system origin relative to projector

    // rotation angles in degrees
    float rot_p = pan - 90;
    float rot_t = tilt;

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

        glMultMatrixd(Ancillary::flattenMat44d(cams[camIdx].getRt44()));

        if (showImVis)
        {
            if (showVirtual)
                pointCloud2ImVis->display(pointSize);
            else
                pointCloudImVis->display(pointSize);
        }
        else
        {
            if (showVirtual)
                pointCloud2Circles->display(pointSize, 0, 1, 0);
            else
            {
                for (int i = 0; i < triangulatedPointCloudCirclesVec.size(); i++)
                {
                    pointCloudCirclesVec[i]->display(pointSize, 1, 0, 0); // red
                    triangulatedPointCloudCirclesVec[i]->display(pointSize, 0, 1, 0); // green
                }

                std::vector<cv::Point3f> points = pointCloudCirclesVec[viewIdx]->getPoints();

                float f = cams[0].getf();
                float CCDWidth_half_mm = 20.0 * 0.02 * 0.5;

                float fProj = cams[2].getf();
                cv::Vec3d projC = cams[2].getC();
                cv::Mat Rt44 = cams[2].getRt44();
                cv::Mat Rt44Inv = cams[2].getRt44Inv();

                for (int ptIdx = 0; ptIdx < points.size(); ptIdx++)
                {
                    glBegin(GL_POINTS);
                        glPointSize(0.5 * pointSize);
                        glColor3f(1.0, 0.0, 0.0);
                        glVertex3f(CCDWidth_half_mm * points[ptIdx].x / points[ptIdx].z, CCDWidth_half_mm * points[ptIdx].y / points[ptIdx].z, CCDWidth_half_mm);
                    glEnd();

                    cv::Vec3d pt = Ancillary::Mat44dTimesVec3dHomog(Rt44, cv::Vec3d(points[ptIdx].x, points[ptIdx].y, points[ptIdx].z));

                    cv::Vec3d projPt(CCDWidth_half_mm * pt[0] / pt[2], CCDWidth_half_mm * pt[1] / pt[2], CCDWidth_half_mm);
                    cv::Vec3d projPtLocal = Ancillary::Mat44dTimesVec3dHomog(Rt44Inv, projPt);
                }
            }
        }

        fittedPlane->display(2, pointSize);

        char* str;
        if (camIdx == 0)
            str = "cam0"; //"cam0";
        else if (camIdx == 1)
            str = "cam1";
        else if (camIdx == 2)
            str = "proj";

        std::stringstream ss;
        ss << "Rendering according to " << string(str);

        displayText(15, 20, 0, 0, 0, ss.str().c_str());

        float offset = 0.02;
        for (int i = 0; i < cams.size(); i++)
        {
            if (viewIdx == 0 && (i == 4 || i == 5))
                continue;

            if (viewIdx == 1 && (i == 4))
                continue;

            float r = 0.5;
            float g = 0.5;
            float b = 0.5;

            if (i == 0 || i == 2)
            { 
                r = 0.0;
                g = 0.0;
                b = 0.0;
            }

            cams[i].displayWorld(r, g, b);
        }

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

	float w = cams[camIdx].getWidth();
    float h = cams[camIdx].getHeight();

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

    double f = cams[camIdx].getf() * scaleFactor;
    double cx = cams[camIdx].getPrincipalPt()[0] * scaleFactor;
    double cy = cams[camIdx].getPrincipalPt()[1] * scaleFactor;

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
    case GLFW_KEY_F2:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            camIdx++;
            if (camIdx > cams.size() - 1)
                camIdx = 0;

            reshape(-1, -1);

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F3:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            camIdx--;
            if (camIdx < 0)
                camIdx = cams.size() - 1;

            reshape(-1, -1);

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F4:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            //showDistances = !showDistances;
            viewIdx++;
            if (viewIdx > 3)
                viewIdx = 0;

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F5:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            showVirtual = !showVirtual;

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F6:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            showImVis = !showImVis;

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
    viewportWidth = w;
    viewportHeight = h;

    showDistances = false;
    showVirtual = false;
    showImVis = false;

    cursorPrevX = 0;
    cursorPrevY = 0;

    step = 0.1;
    arrowKeyStep = 0.001;
    cloudArrowKeyStep = 0.001;
    lockMouse = false;
    fastMove = false;
    fastMult = 10;
    toggleWireframe = false;
    defaultPointSize = 3;
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

    window = glfwCreateWindow(viewportWidth, viewportHeight, "calibrateProjMirror (IMW-CPS TU Vienna / michael.hornacek@gmail.com)", NULL, NULL);
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

    int visImIdx = 0;
    if (argc >= 15)
        visImIdx = parser.get<int>(13);

    string visImPath;
    bool hasVisIm = false;
    if (argc == 16)
    {
        visImPath = parser.get<String>(14);
        hasVisIm = true;
    }

    cv::Size chessboardPatternSize(chessboardDimsX, chessboardDimsY);
    cv::Size circlesPatternSize(circlesDimsX, circlesDimsY);

    std::vector<std::vector<cv::Point3f>> chessboardObjectPts;
    CalibPattern::computeChessboardObjPts(numIms, chessboardSqSize, chessboardPatternSize,
        chessboardObjectPts);

    // compute circles projector points
    std::vector<cv::Point2f> circlesProjPts_;
    cv::Mat projIm = cv::imread(circlesImPath);
    cv::Size projSize = projIm.size();
    cv::Mat projImGray, projImVis;
    projIm.copyTo(projImVis);
    cv::cvtColor(projIm, projImGray, cv::COLOR_BGR2GRAY);
    cv::findCirclesGrid(projImGray, circlesPatternSize, circlesProjPts_, cv::CALIB_CB_ASYMMETRIC_GRID);
    cv::drawChessboardCorners(projImVis, circlesPatternSize, circlesProjPts_, true);
    cv::imshow("projected image (with detected circles)", projImVis);
    cv::waitKey(5);

    cv::Mat visIm;
    if (hasVisIm)
        visIm = cv::imread(visImPath);
    else
        projIm.copyTo(visIm);

    // store copy of circles projector points once per input image
    std::vector<std::vector<cv::Point2f>> circlesProjPts;
    for (int numIm = 0; numIm < numIms; numIm++)
        circlesProjPts.push_back(circlesProjPts_);

    // read in intrinsics and extrinsics of cam0
    FileStorage fsCam0;
    fsCam0.open(cam0Path, FileStorage::READ);

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
        imWidth, imHeight, 0.02));

    // read in intrinsics and extrinsics of cam1
    FileStorage fsCam1;
    fsCam1.open(cam1Path, FileStorage::READ);

    cv::Mat cam1K, cam1R, cam1T, cam1DistCoeffs;
    fsCam1["K"] >> cam1K;
    fsCam1["R"] >> cam1R;
    fsCam1["t"] >> cam1T;
    fsCam1["distCoeffs"] >> cam1DistCoeffs;

    cams.push_back(Camera(
        cam1K, cam1R, cam1T,
        imWidth, imHeight, 0.02));

    // get circles and chessboard image points determine corresponding plane parameters
    std::vector<std::vector<cv::Point2f>> cam0ChessboardImPts, cam0CirclesImPts, cam0CirclesImPtsFlattened;
    cv::Mat cam0ImVis;

    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << cam0ImDir << "\\" << numIm << ".png";
        std::cout << ss1.str() << std::endl;

        cv::Mat imVis_;
        //CalibPattern::findChessboardImPts(cv::imread(ss1.str()), chessboardPatternSize,
        //    outChessboardImPts, outImSize, imVis_,
        //    applyIntrinsics, K, distCoeffs);

        //std::cout << "findChessboardImPts done" << std::endl;

        CalibPattern::findCirclesImPts(cv::imread(ss1.str()), circlesPatternSize,
            cam0CirclesImPts, cv::Size(), imVis_,
            true, cam0K, cam0DistCoeffs);

        std::cout << "findCirclesImPts done" << std::endl;

        if (numIm == visImIdx)
            imVis_.copyTo(cam0ImVis);

        std::stringstream ss2;
        ss2 << "projector calibration image (50% resized)";

        cv::resize(imVis_, imVis_, cv::Size(imVis_.cols * 0.5, imVis_.rows * 0.5));

        cv::imshow(ss2.str(), imVis_);
        cv::waitKey(100);
    }

    double cam0f = cams[0].getf();
    cv::Vec2d cam0PrincipalPt = cams[0].getPrincipalPt();
    for (int i = 0; i < cam0CirclesImPts.size(); i++)
    {
        cam0CirclesImPtsFlattened.push_back(std::vector<cv::Point2f>());
        for (int j = 0; j < cam0CirclesImPts[i].size(); j++)
            cam0CirclesImPtsFlattened[i].push_back(cam0CirclesImPts[i][j]);
    }

    std::vector<std::vector<cv::Point2f>> cam1ChessboardImPts, cam1CirclesImPts, cam1CirclesImPtsFlattened;
    cv::Mat cam1ImVis;

    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << cam1ImDir << "\\" << numIm << ".png";
        std::cout << ss1.str() << std::endl;

        cv::Mat imVis_;
        //CalibPattern::findChessboardImPts(cv::imread(ss1.str()), chessboardPatternSize,
        //    outChessboardImPts, outImSize, imVis_,
        //    applyIntrinsics, K, distCoeffs);

        //std::cout << "findChessboardImPts done" << std::endl;

        CalibPattern::findCirclesImPts(cv::imread(ss1.str()), circlesPatternSize,
            cam1CirclesImPts, cv::Size(), imVis_,
            true, cam1K, cam1DistCoeffs);

        std::cout << "findCirclesImPts done" << std::endl;

        if (numIm == visImIdx)
            imVis_.copyTo(cam1ImVis);

        std::stringstream ss2;
        ss2 << "projector calibration image (50% resized)";

        cv::resize(imVis_, imVis_, cv::Size(imVis_.cols * 0.5, imVis_.rows * 0.5));

        cv::imshow(ss2.str(), imVis_);
        cv::waitKey(100);
    }

    double cam1f = cams[1].getf();
    cv::Vec2d cam1PrincipalPt = cams[1].getPrincipalPt();
    for (int i = 0; i < cam1CirclesImPts.size(); i++)
    {
        cam1CirclesImPtsFlattened.push_back(std::vector<cv::Point2f>());
        for (int j = 0; j < cam1CirclesImPts[i].size(); j++)
            cam1CirclesImPtsFlattened[i].push_back(cam1CirclesImPts[i][j]);
    }

    // compute circles object points by intersecting circle pixel back-projections with RANSAC ground plane
    std::vector<cv::Point3f> triangulatedCirclesObjectPts;
    std::vector<std::vector<cv::Point3f>> circlesObjectPts;
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::vector<cv::Point3f> triangulatedCirclesObjectPts_;
        
        cv::Mat intersectionsHomog(4, numIm, CV_64F);
        cv::triangulatePoints(
            cams[0].getP(), cams[1].getP(),
            cam0CirclesImPtsFlattened.at(numIm), cam1CirclesImPtsFlattened.at(numIm),
            intersectionsHomog);
        
        for (int i = 0; i < intersectionsHomog.cols; i++)
        {
            cv::Vec4d ptHomog(intersectionsHomog.col(i));

            cv::Point3d pt(
                ptHomog[0] / ptHomog[3],
                ptHomog[1] / ptHomog[3],
                ptHomog[2] / ptHomog[3]);

            triangulatedCirclesObjectPts_.push_back(pt);
        }

        for (int i = 0; i < triangulatedCirclesObjectPts_.size(); i++)
            triangulatedCirclesObjectPts.push_back(triangulatedCirclesObjectPts_[i]);
        
        triangulatedPointCloudCirclesVec.push_back(new PointCloud(triangulatedCirclesObjectPts_));
    }

    fittedPlane = new Plane(triangulatedCirclesObjectPts, true);

    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::vector<cv::Point3f> circlesObjectPts_;
        for (int numCircle = 0; numCircle < cam0CirclesImPts.at(numIm).size(); numCircle++)
        {
            cv::Vec3d intersection = fittedPlane->intersect(cams[0].backprojectLocal(cam0CirclesImPts.at(numIm).at(numCircle)));
            circlesObjectPts_.push_back(cv::Point3d(intersection[0], intersection[1], intersection[2]));
        }

        circlesObjectPts.push_back(circlesObjectPts_);
        pointCloudCirclesVec.push_back(new PointCloud(circlesObjectPts_));
    }

    cv::Mat projK = Mat::eye(3, 3, CV_64F);
    projK.at<double>(0, 0) = 2500;
    projK.at<double>(1, 1) = 2500;
    projK.at<double>(0, 2) = projSize.width * 0.5;
    projK.at<double>(1, 2) = projSize.height * 0.5;
    cv::Mat projDistCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<cv::Mat> projRs, projTs;

    // compute projector calibration
    cout << "RMS (projector calibration): " << cv::calibrateCamera(circlesObjectPts, circlesProjPts, projSize,
        projK, projDistCoeffs, projRs, projTs,
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT |
        cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 |
        cv::CALIB_ZERO_TANGENT_DIST) << endl;

    for (int numIm = 0; numIm < numIms; numIm++)
    {
        cv::Mat rot;
        cv::Rodrigues(projRs.at(numIm), rot);

        // write out projector intrinsics + extrinsics
        stringstream ssProj;
        ssProj << outDir << "\\proj_" << numIm << ".yml";
        cv::FileStorage fsProj(ssProj.str(), cv::FileStorage::WRITE);

        fsProj << "K" << projK;
        fsProj << "R" << rot;
        fsProj << "t" << projTs.at(numIm);
        fsProj << "width" << projSize.width;
        fsProj << "height" << projSize.height;
        fsProj << "distCoeffs" << projDistCoeffs;

        fsProj.release();

        Camera proj(
            projK, rot, projTs.at(numIm),
            projSize.width, projSize.height, 0.02);

        float verticalOffset = proj.getf() * targetWidth / projSize.width;
        cout << "verticalOffset 1: " << verticalOffset << endl;
        cv::Mat H;

        cv::Mat virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT;
        Homography::computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(*fittedPlane, cams[0], proj, verticalOffset, virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT);

        Camera transformedProj(
            projK, alignedVirtualProjR, alignedVirtualProjT,
            projSize.width, projSize.height, 0.02);

        Homography::computePlaneInducedHomography(*fittedPlane, proj, transformedProj, H);

        // write out homography
        stringstream ssH;
        ssH << outDir << "\\homography_" << numIm << ".yml";
        std::cout << ssH.str() << std::endl;
        cv::FileStorage fsH(ssH.str(), cv::FileStorage::WRITE);

        fsH << "H" << H;

        fsH.release();
        
        cv::Mat outIm;
        cv::warpPerspective(visIm, outIm, H, projSize);
        cv::imshow("warped image (w.r.t. final virtual projector homography)", outIm);
    }


    // visualization
    {
        int numIm = visImIdx;

        cv::Mat camToProjR, camToProjT;
        cv::Rodrigues(projRs.at(numIm), camToProjR);
        projTs.at(numIm).copyTo(camToProjT);

        cams.push_back(Camera(
            projK, camToProjR, camToProjT,
            projSize.width, projSize.height, 0.02));
        
        cv::resize(cam0ImVis, cam0ImVis, cv::Size(cam0ImVis.cols * 0.5, cam0ImVis.rows * 0.5));
        cv::imshow("cam0ImVis", cam0ImVis);
    }

    glutInit(&argc, argv);
    reshape(-1, -1);

    /* Main event loop */
    while (true)
    {
        glfwPollEvents();

        /* Navigation handling (I could not figure out how to handle key combinations using keyboard callback) */
        if ((glfwGetKey(window, GLFW_KEY_UP) || glfwGetKey(window, 'W')) && (glfwGetKey(window, GLFW_KEY_RIGHT) || glfwGetKey(window, 'D'))) // NE
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, GLFW_KEY_DOWN) || glfwGetKey(window, 'S')) && (glfwGetKey(window, GLFW_KEY_RIGHT) || glfwGetKey(window, 'D'))) // SE
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, GLFW_KEY_DOWN) || glfwGetKey(window, 'S')) && (glfwGetKey(window, GLFW_KEY_LEFT) || glfwGetKey(window, 'A'))) // SW
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if ((glfwGetKey(window, GLFW_KEY_UP) || glfwGetKey(window, 'W')) && (glfwGetKey(window, GLFW_KEY_LEFT) || glfwGetKey(window, 'A'))) // NW
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_UP) || glfwGetKey(window, 'W')) // N
        {
            cloudY -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_RIGHT) || glfwGetKey(window, 'D'))	// E
        {
            cloudX += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_DOWN) || glfwGetKey(window, 'S')) // S
        {
            cloudY += (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }
        else if (glfwGetKey(window, GLFW_KEY_LEFT) || glfwGetKey(window, 'A')) // W
        {
            cloudX -= (fastMove) ? fastMult * cloudArrowKeyStep : cloudArrowKeyStep;

            newEvent = true;
        }

        /* Render the scene */
        display();
    }
}
