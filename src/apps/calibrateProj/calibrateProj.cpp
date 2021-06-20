/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   calibrateProj <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <cam0ImDir> <cam1ImDir> <cam0Path> <cam1Path> <circlesImPath> <verticalNegOffset> [<visImIdx>] [<visIm>]
 *
 *   Example invocation:
 *   calibrateProj 0.0565 4 6 4 11 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out 14 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outRight C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_1.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\acircles_pattern_960x600.png 1.75 3 C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png
 */


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
vector<Camera> cams;
int imWidth, imHeight;

Camera projCam;
int projWidth, projHeight;

int numPatterns;
vector<vector<Mat>> capturedPattern;

cv::Vec3d centroidChessboardObjectPts;

PointCloud * pointCloud;
PointCloud * pointCloudCircles;
PointCloud* pointCloud2Circles;
PointCloud* pointCloudImVis;
PointCloud* pointCloud2ImVis;
Plane * planeVis;

std::vector<Plane> planes;

cv::Vec3d projCamIntersection;
cv::Vec3d projPlaneIntersection;
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
    "{@verticalNegOffset | | ...}"
    "{@visImIdx | | ...}"
    "{@visIm | | ...}"
};

void help()
{
    cout << "calibrate <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <cam0ImDir> <cam1ImDir> <cam0Path> <cam1Path> <circlesImPath> <verticalNegOffset> [<visImIdx>] [<visIm>]\n"
        << endl;
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

        pointCloud->display(pointSize, 0, 0, 1);

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
                pointCloudCircles->display(pointSize, 1, 0, 0);

                std::vector<cv::Point3f> points = pointCloudCircles->getPoints();
                std::vector<cv::Point3f> points2 = pointCloud->getPoints();

                float f = cams[0].getf();
                float CCDWidth_half_mm = 20.0 * 0.01 * 0.5;

                for (int ptIdx = 0; ptIdx < points.size(); ptIdx++)
                {
                    glBegin(GL_POINTS);
                        glPointSize(0.5 * pointSize);
                        glColor3f(1.0, 0.0, 0.0);
                        glVertex3f(CCDWidth_half_mm * points[ptIdx].x / points[ptIdx].z, CCDWidth_half_mm * points[ptIdx].y / points[ptIdx].z, CCDWidth_half_mm);
                    glEnd();

                    glBegin(GL_LINES);
                        glColor3f(0.9, 0.9, 0.9);
                        glVertex3f(0, 0, 0);
                        glVertex3f(points[ptIdx].x, points[ptIdx].y, points[ptIdx].z);
                    glEnd();

                    glBegin(GL_POINTS);
                        glPointSize(0.5 * pointSize);
                        glColor3f(1.0, 0.0, 0.0);
                        glVertex3f(CCDWidth_half_mm * points[ptIdx].x / points[ptIdx].z, CCDWidth_half_mm * points[ptIdx].y / points[ptIdx].z, CCDWidth_half_mm);
                    glEnd();
                }
                   
                for (int ptIdx = 0; ptIdx < points2.size(); ptIdx++)
                {
                    glBegin(GL_POINTS);
                        glPointSize(0.5 * pointSize);
                        glColor3f(0.0, 0.0, 1.0);
                        glVertex3f(CCDWidth_half_mm * points2[ptIdx].x / points2[ptIdx].z, CCDWidth_half_mm * points2[ptIdx].y / points2[ptIdx].z, CCDWidth_half_mm);
                    glEnd();
                }
            }
        }

        planeVis->display(1.5, pointSize);

        float offset = 0.02;
        for (int i = 0; i < cams.size(); i++)
        {
            //if (i != 0)
            //    continue;

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

            char* str;
            if (i == 0)
                str = "cam0";
            else if (i == 1)
                str = "cam1";
            else if (i == 2)
                str = "projector";
            else if (i == 3)
                str = "projector virtual (downward)";
            else if (i == 4)
                str = "projector virtual (final)";
            else if (i == 5)
                str = "cam0 virtual (downward)";

            cv::Vec3d C = cams[i].getC();

            if (camIdx == i)
            {
                std::stringstream ss;
                ss << "Rendering according to " << string(str);

                displayText(15, 20, 0, 0, 0, ss.str().c_str());
            }

            float additionalOffset = 0;
            if (i == 5)
                additionalOffset = 0.03;

            displayText(C[0] + offset, C[1] + offset, C[2] - offset + additionalOffset, 0, 0, 0, str);
        }

        displayText(
            centroidChessboardObjectPts[0] + offset,
            centroidChessboardObjectPts[1] + offset,
            centroidChessboardObjectPts[2] - offset,
            0, 0, 0, "ground plane");

        if (showDistances)
        {
            //cv::Vec3d camC = cams[0].getC();

            //glBegin(GL_LINES);
            //glVertex3f(camC[0], camC[1], camC[2]);
            //glVertex3f(virtualCamPlaneIntersection[0], virtualCamPlaneIntersection[1], virtualCamPlaneIntersection[2]);
            //glEnd();

            //double distVirtualCamPlaneIntersection = sqrt(
            //    (camC - virtualCamPlaneIntersection).dot(camC - virtualCamPlaneIntersection));

            //std::stringstream ss0;
            //ss0 << distVirtualCamPlaneIntersection << " m";

            //displayText(
            //    virtualCamPlaneIntersection[0] + (camC - virtualCamPlaneIntersection)[0] * 0.5 + offset,
            //    virtualCamPlaneIntersection[1] + (camC - virtualCamPlaneIntersection)[1] * 0.5 + offset,
            //    virtualCamPlaneIntersection[2] + (camC - virtualCamPlaneIntersection)[2] * 0.5 - offset,
            //    0, 0, 0, ss0.str().c_str());

            cv::Vec3d projC = cams[2].getC();

            glBegin(GL_LINES);
            glVertex3f(projC[0], projC[1], projC[2]);
            glVertex3f(projPlaneIntersection[0], projPlaneIntersection[1], projPlaneIntersection[2]);
            glEnd();

            double distProjPlaneIntersection = sqrt(
                (projC - projPlaneIntersection).dot(projC - projPlaneIntersection));

            std::stringstream ss1;
            ss1 << distProjPlaneIntersection << " m";

            displayText(
                projPlaneIntersection[0] + (projC - projPlaneIntersection)[0] * 0.5 + offset,
                projPlaneIntersection[1] + (projC - projPlaneIntersection)[1] * 0.5 + offset,
                projPlaneIntersection[2] + (projC - projPlaneIntersection)[2] * 0.5 - offset,
                0, 0, 0, ss1.str().c_str());

            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(projC[0], projC[1], projC[2]);
            glEnd();

            double distCamLeftProj = sqrt(projC.dot(projC));

            std::stringstream ss2;
            ss2 << distCamLeftProj << " m";

            displayText(projC[0] * 0.5 + offset, projC[1] * 0.5 + offset, projC[2] * 0.5 - offset,
                0, 0, 0, ss2.str().c_str());


            cv::Vec3d virtualProjC = cams[4].getC();

            double distVirtualProjPlaneIntersection = sqrt(
                (virtualProjC - projPlaneIntersection).dot(virtualProjC - projPlaneIntersection));

            glBegin(GL_LINES);
            glVertex3f(virtualProjC[0], virtualProjC[1], virtualProjC[2]);
            glVertex3f(projPlaneIntersection[0], projPlaneIntersection[1], projPlaneIntersection[2]);
            glEnd();

            std::stringstream ss3;
            ss3 << distVirtualProjPlaneIntersection << " m";

            displayText(
                projPlaneIntersection[0] + (virtualProjC - projPlaneIntersection)[0] * 0.5 + offset,
                projPlaneIntersection[1] + (virtualProjC - projPlaneIntersection)[1] * 0.5 + offset,
                projPlaneIntersection[2] + (virtualProjC - projPlaneIntersection)[2] * 0.5 - offset,
                0, 0, 0, ss3.str().c_str());
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
                          -2*cx/w+1,  2*cy/h-1, -(farPlane+nearPlane)/(farPlane-nearPlane), -1,
                           0,         0,        -2*farPlane*nearPlane/(farPlane-nearPlane),  0};
	
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
            showDistances = !showDistances;

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
    centroidChessboardObjectPts = cv::Vec3d(0, 0, 0);

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

    window = glfwCreateWindow(viewportWidth, viewportHeight, "calibrateProj (IMW-CPS TU Vienna / michael.hornacek@gmail.com)", NULL, NULL);
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
    float verticalOffset = -parser.get<float>(12);

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
        imWidth, imHeight, 0.01));

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
        imWidth, imHeight, 0.01));

    // get circles and chessboard image points determine corresponding plane parameters
    std::vector<std::vector<cv::Point2f>> cam0ChessboardImPts, cam0CirclesImPts;
    CalibPattern::findChessboardAndCirclesImPts(cam0ImDir, numIms,
        chessboardPatternSize, circlesPatternSize,
        cam0ChessboardImPts, cam0CirclesImPts, cv::Size(),
        true, cam0K, cam0DistCoeffs);

    std::vector<std::vector<cv::Point2f>> cam1ChessboardImPts, cam1CirclesImPts;
    CalibPattern::findChessboardAndCirclesImPts(cam1ImDir, numIms,
        chessboardPatternSize, circlesPatternSize,
        cam1ChessboardImPts, cam1CirclesImPts, cv::Size(),
        true, cam1K, cam1DistCoeffs);

    std::vector<cv::Mat> planeRs, planeTs;

    // use not to calibrate camera (we provide camera parameters and keep them fixed), but to obtain plane for each image in cam coordinate frame
    cout << "RMS (camera resection/plane recovery): " << cv::calibrateCamera(chessboardObjectPts, cam0ChessboardImPts, camSize,
        cam0K, cam0DistCoeffs, planeRs, planeTs,
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_FOCAL_LENGTH |
        cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 |
        cv::CALIB_ZERO_TANGENT_DIST) << endl;

    // compute circles object points by intersecting circle pixel back-projections with ground plane
    std::vector<std::vector<cv::Point3f>> circlesObjectPts;
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        cv::Mat planeRot;
        cv::Rodrigues(planeRs.at(numIm), planeRot);

        cv::Mat planeRigid = cv::Mat::eye(cv::Size(4, 4), CV_64F);
        for (int y = 0; y < 3; y++)
            for (int x = 0; x < 3; x++)
                planeRigid.at<double>(y, x) = planeRot.at<double>(y, x);

        planeRigid.at<double>(0, 3) = planeTs.at(numIm).at<double>(0, 0);
        planeRigid.at<double>(1, 3) = planeTs.at(numIm).at<double>(1, 0);
        planeRigid.at<double>(2, 3) = planeTs.at(numIm).at<double>(2, 0);

        // compute ground plane w.r.t. checkerboard pattern
        Plane imPlane(planeRigid);

        stringstream ssPlane;
        ssPlane << outDir << "\\plane_" << numIm << ".yml";
        cv::FileStorage fsPlane(ssPlane.str(), cv::FileStorage::WRITE);

        fsPlane << "Rt" << planeRigid;

        std::vector<cv::Point3f> circlesObjectPts_;
        if (false)
        {
            cv::Mat intersectionsHomog;
            cv::triangulatePoints(cams[0].getP(), cams[1].getP(), cam0CirclesImPts.at(numIm), cam1CirclesImPts.at(numIm), intersectionsHomog);

            for (int i = 0; i < intersectionsHomog.cols; i++)
            {
                cv::Vec4d ptHomog(intersectionsHomog.col(i));

                cv::Point3d pt(
                    ptHomog[0] / ptHomog[3],
                    ptHomog[1] / ptHomog[3],
                    ptHomog[2] / ptHomog[3]);

                circlesObjectPts_.push_back(pt);
            }
        }
        else
        {
            for (int numCircle = 0; numCircle < cam0CirclesImPts.at(numIm).size(); numCircle++)
            {
                cv::Vec3d intersection = imPlane.intersect(cams[0].backprojectLocal(cam0CirclesImPts.at(numIm).at(numCircle)));
                circlesObjectPts_.push_back(cv::Point3d(intersection[0], intersection[1], intersection[2]));
            }
        }

        circlesObjectPts.push_back(circlesObjectPts_);
        planes.push_back(Plane(planeRigid));

        // for visualization
        if (numIm == visImIdx)
        {
            pointCloudCircles = new PointCloud(circlesObjectPts_);
            planeVis = new Plane(planeRigid);
        }
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
            projSize.width, projSize.height, 0.01);

        cv::Mat H;

        cv::Mat virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT;
        Homography::computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(planes[numIm], cams[0], proj, verticalOffset, virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT);

        Camera transformedProj(
            projK, alignedVirtualProjR, alignedVirtualProjT,
            projSize.width, projSize.height, 0.01);

        Homography::computePlaneInducedHomography(planes[numIm], proj, transformedProj, H);

        // write out homography
        stringstream ssH;
        ssH << outDir << "\\homography_" << numIm << ".yml";
        cv::FileStorage fsH(ssH.str(), cv::FileStorage::WRITE);

        fsH << "H" << H;

        fsH.release();
    }


    // visualization
    {
        int numIm = visImIdx;

        cv::Mat camToProjR, camToProjT;
        cv::Rodrigues(projRs.at(numIm), camToProjR);
        projTs.at(numIm).copyTo(camToProjT);

        cams.push_back(Camera(
            projK, camToProjR, camToProjT,
            projSize.width, projSize.height, 0.01));

        cv::Mat planeRigid;
        planes[numIm].getRigid(planeRigid);

        std::vector<cv::Point3f> transformedChessboardObjectPts;
        for (int i = 0; i < chessboardObjectPts[0].size(); i++)
        {
            cv::Vec3d pt(chessboardObjectPts[0][i].x, chessboardObjectPts[0][i].y, chessboardObjectPts[0][i].z);
            cv::Vec3d outPt = Ancillary::Mat44dTimesVec3dHomog(planeRigid, pt);

            transformedChessboardObjectPts.push_back(cv::Point3f(outPt[0], outPt[1], outPt[2]));
            centroidChessboardObjectPts += outPt;
        }
        centroidChessboardObjectPts[0] /= chessboardObjectPts[0].size();
        centroidChessboardObjectPts[1] /= chessboardObjectPts[0].size();
        centroidChessboardObjectPts[2] /= chessboardObjectPts[0].size();

        pointCloud = new PointCloud(transformedChessboardObjectPts);
        

        Plane planeProjLocal(planes[numIm].getNormal(), planes[numIm].getDistance());
        planeProjLocal.rigidTransform(cams[2].getRt44());

        cv::Vec3d projPlaneIntersectionLocal = planeProjLocal.intersect(
            cams[2].backprojectLocal(cv::Point2f(projSize.width * 0.5, projSize.height * 0.5)));

        projPlaneIntersection = Ancillary::Mat44dTimesVec3dHomog(cams[2].getRt44Inv(), projPlaneIntersectionLocal);

        cv::Vec3d projCamIntersectionLocal = planes[numIm].intersect(
            cams[0].backprojectLocal(cv::Point2f(camSize.width * 0.5, camSize.height * 0.5)));

        projCamIntersection = Ancillary::Mat44dTimesVec3dHomog(cams[2].getRt44Inv(), projCamIntersectionLocal);

        cv::Mat H, virtualProjR, virtualProjT;
        Homography::computeBirdsEyeViewHomography(planes[numIm], cams[2], H, virtualProjR, virtualProjT);
        cams.push_back(Camera(
            projK, virtualProjR, virtualProjT,
            projSize.width, projSize.height, 0.01));

        cv::Mat outIm;
        //cv::warpPerspective(visIm, outIm, H, projSize);

        //cv::imshow("warped image (w.r.t. virtual projector homography)", outIm);
        //cv::waitKey(5);

        cv::Mat virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT;
        Homography::computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(planes[numIm], cams[0], cams[2],
            verticalOffset, virtualCamR, virtualCamT, alignedVirtualProjR, alignedVirtualProjT);

        cams.push_back(Camera(
            projK, alignedVirtualProjR, alignedVirtualProjT,
            projSize.width, projSize.height, 0.01));

        cams.push_back(Camera(
            cam0K, virtualCamR, virtualCamT,
            camSize.width, camSize.height, 0.01));

        Homography::computePlaneInducedHomography(planes[numIm], cams[2], cams[4], H);

        cv::warpPerspective(visIm, outIm, H, projSize);

        cv::imshow("warped image (w.r.t. final virtual projector homography)", outIm);
        cv::waitKey(5);

        vector<Point3f> imVisPts, imVisColors;
        for (int y = 0; y < projSize.height; y += 3)
        {
            for (int x = 0; x < projSize.width; x += 3)
            {
                cv::Vec3d intersectionLocal = planeProjLocal.intersect(cams[2].backprojectLocal(cv::Point2f(x, y)));
                cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[2].getRt44Inv(), intersectionLocal);
                imVisPts.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));

                cv::Vec3b color = visIm.at<cv::Vec3b>(cv::Point2f(x, y));
                imVisColors.push_back(cv::Point3d(color[2] / 255., color[1] / 255., color[0] / 255.));
            }
        }
        pointCloudImVis = new PointCloud(imVisPts, imVisColors);

        Plane planeProjVirtualLocal(planes[numIm].getNormal(), planes[numIm].getDistance());
        planeProjVirtualLocal.rigidTransform(cams[4].getRt44());

        vector<Point3f> circlesObjectPtsVirtual_;
        for (int numCircle = 0; numCircle < circlesProjPts.at(numIm).size(); numCircle++)
        {
            cv::Vec3d intersectionLocal = planeProjVirtualLocal.intersect(cams[4].backprojectLocal(circlesProjPts.at(numIm).at(numCircle)));
            cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[4].getRt44Inv(), intersectionLocal);
            circlesObjectPtsVirtual_.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));
        }
        pointCloud2Circles = new PointCloud(circlesObjectPtsVirtual_);

        vector<Point3f> imVis2Pts, imVis2Colors;
        for (int y = 0; y < projSize.height; y += 3)
        {
            for (int x = 0; x < projSize.width; x += 3)
            {
                cv::Vec3d intersectionLocal = planeProjLocal.intersect(cams[2].backprojectLocal(cv::Point2f(x, y)));
                cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[2].getRt44Inv(), intersectionLocal);
                imVis2Pts.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));

                cv::Vec3b color = outIm.at<cv::Vec3b>(cv::Point2f(x, y));
                imVis2Colors.push_back(cv::Point3d(color[2] / 255., color[1] / 255., color[0] / 255.));
            }
        }
        pointCloud2ImVis = new PointCloud(imVis2Pts, imVis2Colors);

        Plane planCamVirtualLocal(planes[numIm].getNormal(), planes[numIm].getDistance());
        planCamVirtualLocal.rigidTransform(cams[5].getRt44());

        cv::Vec3d virtualCamPlaneIntersectionLocal = planCamVirtualLocal.intersect(
            cams[5].backprojectLocal(cv::Point2f(camSize.width * 0.5, camSize.height * 0.5)));

        virtualCamPlaneIntersection = Ancillary::Mat44dTimesVec3dHomog(cams[5].getRt44Inv(), virtualCamPlaneIntersectionLocal);
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
