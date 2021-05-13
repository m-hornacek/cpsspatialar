/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   calibrateProj <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <imDir> <cam0Path> <circlesImPath> <verticalNegOffset> [<visImIdx>] [<visIm>]
 *
 *   Example invocation:
 *   calibrateProj 0.0565 4 6 4 11 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out 14 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\acircles_pattern_960x600.png 1.75 3 C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png
 */


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "headers.h"
#include "Camera.h"
#include "PointCloud.h"
#include "Plane.h"

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

PointCloud * pointCloud;
PointCloud * pointCloudCircles;
PointCloud* pointCloud2Circles;
PointCloud* pointCloudImVis;
PointCloud* pointCloud2ImVis;
Plane * planeVis;

std::vector<Plane> planes;

cv::Vec3d projCamIntersection;
cv::Vec3d projPlaneIntersection;

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
    "{@imDir | | ...}"
    "{@cam0Path | | ...}"
    "{@circlesImPath | | ...}"
    "{@verticalNegOffset | | ...}"
    "{@visImIdx | | ...}"
    "{@visIm | | ...}"
};

void help()
{
    cout << "./calibrate <boardSqSize> <boardDimsX> <boardDimsY> <circlesDimsX> <circlesDimsY> <outDir> <numIms> <imDir> <cam0Path> <circlesImPath> <verticalNegOffset> [<visImIdx>] [<visIm>]\n"
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
                pointCloudCircles->display(pointSize, 1, 0, 0);
        }

        planeVis->display(2.5, pointSize);

        float offset = 0.02;
        for (int i = 0; i < cams.size(); i++)
        {
            float r = 0.5;
            float g = 0.5;
            float b = 0.5;

            cams[i].displayWorld(r, g, b);

            char* str;
            if (i == 0)
                str = "camera";
            else if (i == 1)
                str = "projector";
            else if (i == 2)
                str = "virtual projector";
            else if (i == 3)
                str = "virtual projector (final)";

            cv::Vec3d C = cams[i].getC();

            displayText(C[0] + offset, C[1] + offset, C[2] - offset, 1.0, 0.0, 0.0, str);
        }

        if (showDistances)
        {
            cv::Vec3d projC = cams[1].getC();

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
                1.0, 0.0, 0.0, ss1.str().c_str());

            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(projC[0], projC[1], projC[2]);
            glEnd();

            double distCamLeftProj = sqrt(projC.dot(projC));

            std::stringstream ss2;
            ss2 << distCamLeftProj << " m";

            displayText(projC[0] * 0.5 + offset, projC[1] * 0.5 + offset, projC[2] * 0.5 - offset,
                1.0, 0.0, 0.0, ss2.str().c_str());


            cv::Vec3d virtualProjC = cams[3].getC();

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
                1.0, 0.0, 0.0, ss3.str().c_str());
        }

        cv::Vec3d planePt = planeVis->intersect(cv::Vec3d(0, 0, 1));

        displayText(
            planePt[0] + offset,
            planePt[1] + offset,
            planePt[2] - offset,
            1.0, 0.0, 0.0, "ground plane");


        

        glFlush();
        glfwSwapBuffers(window);

        newEvent = false;
    }
}

void reshape(int width, int height)
{
    float scaleFactor = 1.0;

	float w = cams[camIdx].getWidth();
    float h = cams[camIdx].getHeight();

    //if (w > width)
    //    scaleFactor = 0.5;

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
            showVirtual = !showVirtual;

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F4:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            showImVis = !showImVis;

            newEvent = true;
            display();
        }
        break;
    case GLFW_KEY_F5:
        if (action == GLFW_RELEASE || action == GLFW_REPEAT)
        {
            showDistances = !showDistances;

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

void findChessboardImPts(cv::Mat& im, Size chessboardPatternSize,
    vector<vector<Point2f>>& outChessboardImPts, Size& outImSize, cv::Mat& outImVis,
    bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    cv::Mat imUndistort, imGrayUndistort;

    if (applyIntrinsics)
    {
        cv::undistort(im, imUndistort, K, distCoeffs);
    }
    else
        im.copyTo(imUndistort);

    if (outImVis.size != imUndistort.size)
        imUndistort.copyTo(outImVis);

    cv::cvtColor(imUndistort, imGrayUndistort, COLOR_BGR2GRAY);
    // cv::threshold(candidateIm, candidateImThresh, 100, 255, THRESH_BINARY);

    outImSize = Size(im.cols, im.rows);

    vector<Point2f> candidateChessboardImPts;
    if (findChessboardCorners(imGrayUndistort, chessboardPatternSize, candidateChessboardImPts, CALIB_CB_ADAPTIVE_THRESH))
    {
        cv::cornerSubPix(imGrayUndistort, candidateChessboardImPts, cv::Size(5, 5), cv::Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));

        drawChessboardCorners(outImVis, chessboardPatternSize, candidateChessboardImPts, true);
        outChessboardImPts.push_back(candidateChessboardImPts);
    }
}

void findCirclesImPts(cv::Mat& im, Size circlesPatternSize,
    vector<vector<Point2f>>& outCirclesImPts, Size& outImSize, cv::Mat& outImVis,
    bool applyIntrinsics = true, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    cv::Mat imUndistort, imGrayUndistort, imGrayUndistortInvert;

    if (applyIntrinsics)
    {
        cv::undistort(im, imUndistort, K, distCoeffs);
    }
    else
        im.copyTo(imUndistort);

    if (outImVis.size != imUndistort.size)
        imUndistort.copyTo(outImVis);

    cv::cvtColor(imUndistort, imGrayUndistort, COLOR_BGR2GRAY);
    cv::bitwise_not(imGrayUndistort, imGrayUndistortInvert);
    // cv::threshold(candidateIm, candidateImThresh, 100, 255, THRESH_BINARY);

    outImSize = Size(im.cols, im.rows);

    vector<Point2f> candidateCirclesImPts;
    if (findCirclesGrid(imGrayUndistortInvert, circlesPatternSize, candidateCirclesImPts, CALIB_CB_ASYMMETRIC_GRID))
    {
        //cv::cornerSubPix(imGrayUndistort, candidateCirclesImPts, cv::Size(5, 5), cv::Size(-1, -1),
        //    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));

        drawChessboardCorners(outImVis, circlesPatternSize, candidateCirclesImPts, true);
        outCirclesImPts.push_back(candidateCirclesImPts);
    }
}

void findChessboardImPts(String inDir, int numIms, Size chessboardPatternSize,
    vector<vector<Point2f>>& outChessboardImPts, Size& outImSize,
    bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    for (int numIm = 1; numIm <= numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << inDir << "\\" << numIm << ".png";
        cout << ss1.str() << endl;

        cv::Mat im = imread(ss1.str());

        cv::Mat imVis;
        findChessboardImPts(im, chessboardPatternSize,
            outChessboardImPts, outImSize, imVis,
            applyIntrinsics, K, distCoeffs);

        std::stringstream ss2;
        ss2 << "camera calibration image (50% resized)";

        cv::resize(imVis, imVis, cv::Size(imVis.cols * 0.5, imVis.rows * 0.5));

        cv::imshow(ss2.str(), imVis);
        cv::waitKey(10);
    }
}

void findChessboardAndCirclesImPts(String inDir, int numIms, Size chessboardPatternSize, Size circlesPatternSize,
    vector<vector<Point2f>>& outChessboardImPts, vector<vector<Point2f>>& outCirclesImPts, Size& outImSize,
    bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << inDir << "\\" << numIm << ".png";
        cout << ss1.str() << endl;

        cv::Mat im = imread(ss1.str());

        cv::Mat imVis;
        findChessboardImPts(im, chessboardPatternSize,
            outChessboardImPts, outImSize, imVis,
            applyIntrinsics, K, distCoeffs);

        findCirclesImPts(im, circlesPatternSize,
            outCirclesImPts, outImSize, imVis,
            applyIntrinsics, K, distCoeffs);

        std::stringstream ss2;
        ss2 << "projector calibration image (50% resized)";

        cv::resize(imVis, imVis, cv::Size(imVis.cols * 0.5, imVis.rows * 0.5));

        cv::imshow(ss2.str(), imVis);
        cv::waitKey(5);
    }
}

void computeBirdsEyeViewHomography(Plane& plane, Camera& proj, cv::Mat& outH, cv::Mat& outR, cv::Mat& outT)
{
    cv::Mat cvProjRigidGlobal = proj.getRt44();
    Eigen::Matrix4d eigenCamRigidGlobal;
    cv::cv2eigen(cvProjRigidGlobal, eigenCamRigidGlobal);

    // express plane in camera coordinate frame of proj
    Plane planeProjLocal(plane.getNormal(), plane.getDistance());
    planeProjLocal.rigidTransform(cvProjRigidGlobal);

    cv::Vec3d cvN = planeProjLocal.getNormal();
    cv::Vec3d cvPt = cvN * planeProjLocal.getDistance();

    Eigen::Vector3d n(cvN[0], cvN[1], cvN[2]);
    Eigen::Vector3d pt(cvPt[0], cvPt[1], cvPt[2]);

    n /= n.norm(); // sanity check...

    // intersect back-projection of cam image plane center point with plane
    cv::Vec3d cvCenterPt = planeProjLocal.intersect(
        proj.backprojectLocal(cv::Point2f(proj.getWidth() * 0.5, proj.getHeight() * 0.5)));
    Eigen::Vector3d centerPt(cvCenterPt[0], cvCenterPt[1], cvCenterPt[2]);

    double distToPlane = std::sqrt(cvCenterPt.dot(cvCenterPt));

    // compute minimal arc-length rotation from opticalAxis to -planeNormal
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), -n);
    Eigen::Matrix3d invR = q.toRotationMatrix();

    // compute camera center C of virtual camera
    Eigen::Vector3d C = centerPt + distToPlane * n;

    // compute (R, t) of virtual camera, relative to camera coordinate frame of input camera
    Eigen::Matrix3d R = invR.inverse();
    Eigen::Vector3d t = -R * C;

    cv::Mat cvK = proj.getK();
    cv::Mat cvKInv = proj.getKInv();

    Eigen::Matrix3d K, KInv;
    cv::cv2eigen(cvK, K);
    cv::cv2eigen(cvKInv, KInv);

    // compute plane-induced homography
    float d = -n.transpose() * centerPt;
    Eigen::Matrix3d H = K * (R - t * n.transpose() / d) * KInv;
    H = H / H(2, 2);

    Eigen::Matrix3d HInv = H.inverse();
    cv::eigen2cv(HInv, outH);

    cv::Mat cvR;
    cv::eigen2cv(R, cvR);

    cv::Mat cvVirtualProjRigidLocal = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            cvVirtualProjRigidLocal.at<double>(y, x) = cvR.at<double>(y, x);

    cvVirtualProjRigidLocal.at<double>(0, 3) = t[0];
    cvVirtualProjRigidLocal.at<double>(1, 3) = t[1];
    cvVirtualProjRigidLocal.at<double>(2, 3) = t[2];

    // express virutal camera in global coordinate frame
    cv::Mat cvVirtualProjRigidGlobal;
    cv::gemm(cvVirtualProjRigidLocal, cvProjRigidGlobal, 1.0, cv::Mat(), 0.0, cvVirtualProjRigidGlobal);

    outR = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            outR.at<double>(y, x) = cvVirtualProjRigidGlobal.at<double>(y, x);

    outT = cv::Mat::zeros(3, 1, CV_64F);
    outT.at<double>(0, 0) = cvVirtualProjRigidGlobal.at<double>(0, 3);
    outT.at<double>(1, 0) = cvVirtualProjRigidGlobal.at<double>(1, 3);
    outT.at<double>(2, 0) = cvVirtualProjRigidGlobal.at<double>(2, 3);
}


void computeBirdsEyeViewVirtualCam(Plane& plane, Camera& cam, float verticalOffset, cv::Mat& outVirtualCamR, cv::Mat& outVirtualCamT)
{
    cv::Mat cvCamRigidGlobal = cam.getRt44();
    Eigen::Matrix4d eigenCamRigidGlobal;
    cv::cv2eigen(cvCamRigidGlobal, eigenCamRigidGlobal);

    // express plane in camera coordinate frame of cam
    Plane planeProjLocal(plane.getNormal(), plane.getDistance());
    planeProjLocal.rigidTransform(cvCamRigidGlobal);

    cv::Vec3d cvN = planeProjLocal.getNormal();

    Eigen::Vector3d n(cvN[0], cvN[1], cvN[2]);

    n /= n.norm(); // sanity check...

    // intersect back-projection of cam image plane center point with plane
    cv::Vec3d cvCenterPt = planeProjLocal.intersect(
        cam.backprojectLocal(cv::Point2f(cam.getWidth() * 0.5, cam.getHeight() * 0.5)));
    Eigen::Vector3d centerPt(cvCenterPt[0], cvCenterPt[1], cvCenterPt[2]);

    double distToPlane = std::sqrt(cvCenterPt.dot(cvCenterPt));

    // compute minimal arc-length rotation from opticalAxis to -planeNormal
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), -n);
    Eigen::Matrix3d invR = q.toRotationMatrix();

    // compute camera center C of virtual camera
    Eigen::Vector3d C = centerPt + (distToPlane + verticalOffset) * n;

    // compute (R, t) of virtual camera, relative to camera coordinate frame of input camera
    Eigen::Matrix3d R = invR.inverse();
    Eigen::Vector3d t = -R * C;

    cv::Mat cvR;
    cv::eigen2cv(R, cvR);

    cv::Mat cvVirtualCamRigidLocal = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            cvVirtualCamRigidLocal.at<double>(y, x) = cvR.at<double>(y, x);

    cvVirtualCamRigidLocal.at<double>(0, 3) = t[0];
    cvVirtualCamRigidLocal.at<double>(1, 3) = t[1];
    cvVirtualCamRigidLocal.at<double>(2, 3) = t[2];

    // express virutal camera in global coordinate frame
    cv::Mat cvVirtualCamRigidGlobal;
    cv::gemm(cvVirtualCamRigidLocal, cvCamRigidGlobal, 1.0, cv::Mat(), 0.0, cvVirtualCamRigidGlobal);

    outVirtualCamR = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            outVirtualCamR.at<double>(y, x) = cvVirtualCamRigidGlobal.at<double>(y, x);

    outVirtualCamT = cv::Mat::zeros(3, 1, CV_64F);
    outVirtualCamT.at<double>(0, 0) = cvVirtualCamRigidGlobal.at<double>(0, 3);
    outVirtualCamT.at<double>(1, 0) = cvVirtualCamRigidGlobal.at<double>(1, 3);
    outVirtualCamT.at<double>(2, 0) = cvVirtualCamRigidGlobal.at<double>(2, 3);
}

void computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(Plane& plane, Camera& cam, Camera& proj, float verticalOffset, cv::Mat& outProjR, cv::Mat& outProjT)
{
    cv::Mat cvVirtualCamR, cvVirtualCamT;
    computeBirdsEyeViewVirtualCam(plane, cam, 0, cvVirtualCamR, cvVirtualCamT);

    cv::Mat cvVirtualProjR, cvVirtualProjT;
    computeBirdsEyeViewVirtualCam(plane, proj, verticalOffset, cvVirtualProjR, cvVirtualProjT);

    Eigen::Matrix3d virtualCamR;
    cv::cv2eigen(cvVirtualCamR, virtualCamR);

    Eigen::Matrix3d virtualProjR;
    cv::cv2eigen(cvVirtualProjR, virtualProjR);

    Eigen::Vector3d virtualProjT(
        cvVirtualProjT.at<double>(0, 0),
        cvVirtualProjT.at<double>(1, 0),
        cvVirtualProjT.at<double>(2, 0));

    Eigen::Vector3d virtualProjC = -virtualProjR.inverse() * virtualProjT;

    // recompute virtualProjT w.r.t. virtualProjC and virtualCamR
    virtualProjT = -virtualCamR * virtualProjC;

    cvVirtualCamR.copyTo(outProjR);

    outProjT = cv::Mat::zeros(3, 1, CV_64F);
    outProjT.at<double>(0, 0) = virtualProjT[0];
    outProjT.at<double>(1, 0) = virtualProjT[1];
    outProjT.at<double>(2, 0) = virtualProjT[2];
}

void computePlaneInducedHomography(Plane& plane, Camera& cam0, Camera& cam1, cv::Mat& outH)
{
    cv::Mat cvCam0RigidGlobal = cam0.getRt44();
    Eigen::Matrix4d eigenCam0RigidGlobal;
    cv::cv2eigen(cvCam0RigidGlobal, eigenCam0RigidGlobal);

    cv::Mat cvCam1RigidGlobal = cam1.getRt44();
    Eigen::Matrix4d eigenCam1RigidGlobal;
    cv::cv2eigen(cvCam1RigidGlobal, eigenCam1RigidGlobal);

    Eigen::Matrix4d eigenCam1RigidLocal = eigenCam1RigidGlobal * eigenCam0RigidGlobal.inverse();

    Eigen::Matrix3d R;
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            R(y, x) = eigenCam1RigidLocal(y, x);

    Eigen::Vector3d t(
        eigenCam1RigidLocal(0, 3),
        eigenCam1RigidLocal(1, 3),
        eigenCam1RigidLocal(2, 3));

    // express plane in camera coordinate frame of cam0
    Plane planeCam0Local(plane.getNormal(), plane.getDistance());
    planeCam0Local.rigidTransform(cvCam0RigidGlobal);

    cv::Vec3d cvN = planeCam0Local.getNormal();
    Eigen::Vector3d n(cvN[0], cvN[1], cvN[2]);

    n /= n.norm(); // sanity check...

    // intersect back-projection of cam image plane center point with plane
    cv::Vec3d cvCenterPt = planeCam0Local.intersect(
        cam0.backprojectLocal(cv::Point2f(cam0.getWidth() * 0.5, cam0.getHeight() * 0.5)));
    Eigen::Vector3d centerPt(cvCenterPt[0], cvCenterPt[1], cvCenterPt[2]);

    cv::Mat cvK = cam0.getK();
    cv::Mat cvKInv = cam1.getKInv();

    Eigen::Matrix3d K, KInv;
    cv::cv2eigen(cvK, K);
    cv::cv2eigen(cvKInv, KInv);

    // compute plane-induced homography
    float d = -n.transpose() * centerPt;
    Eigen::Matrix3d H = K * (R - t * n.transpose() / d) * KInv;
    H = H / H(2, 2);

    Eigen::Matrix3d HInv = H.inverse();
    cv::eigen2cv(HInv, outH);
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 12)
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
    String imDir = parser.get<String>(7);
    String camPath = parser.get<String>(8);
    String circlesImPath = parser.get<String>(9);
    float verticalOffset = -parser.get<float>(10);

    int visImIdx = 0;
    if (argc >= 13)
        visImIdx = parser.get<int>(11);

    string visImPath;
    bool hasVisIm = false;
    if (argc == 14)
    {
        visImPath = parser.get<String>(12);
        std::cout << visImPath << std::endl;
        hasVisIm = true;
    }

    cv::Size chessboardPatternSize(chessboardDimsX, chessboardDimsY);
    cv::Size circlesPatternSize(circlesDimsX, circlesDimsY);


    // compute chessboard object points
    std::vector<cv::Point3f> chessboardObjectPts_;
    for (int i = 0; i < chessboardPatternSize.height; i++)
        for (int j = 0; j < chessboardPatternSize.width; j++)
            chessboardObjectPts_.push_back(
                cv::Point3f(float(j * chessboardSqSize), float(i * chessboardSqSize), 0));

    // store copy of chessboard object points once per input image
    std::vector<std::vector<cv::Point3f>> chessboardObjectPts;
    for (int numIm = 0; numIm < numIms; numIm++)
        chessboardObjectPts.push_back(chessboardObjectPts_);


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

    // read in intrinsics and extrinsics of cam
    FileStorage fs;
    fs.open(camPath, FileStorage::READ);

    cv::Mat camK, camR, camT, camDistCoeffs;
    fs["K"] >> camK;
    fs["R"] >> camR;
    fs["t"] >> camT;
    fs["distCoeffs"] >> camDistCoeffs;

    int imWidth, imHeight;
    fs["width"] >> imWidth;
    fs["height"] >> imHeight;
    cv::Size camSize(imWidth, imHeight);

    init(imWidth, imHeight);

    cams.push_back(Camera(
        camK, camR, camT,
        imWidth, imHeight, 0.015));

    // get circles and chessboard image points determine corresponding plane parameters
    std::vector<std::vector<cv::Point2f>> camChessboardImPts, camCirclesImPts;
    findChessboardAndCirclesImPts(imDir, numIms,
        chessboardPatternSize, circlesPatternSize,
        camChessboardImPts, camCirclesImPts, cv::Size(),
        true, camK, camDistCoeffs);

    std::vector<cv::Mat> planeRs, planeTs;

    // use not to calibrate camera (we provide camera parameters and keep them fixed), but to obtain plane for each image in cam coordinate frame
    cout << "RMS (camera resection/plane recovery): " << cv::calibrateCamera(chessboardObjectPts, camChessboardImPts, camSize,
        camK, camDistCoeffs, planeRs, planeTs,
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

        vector<Point3f> circlesObjectPts_;
        for (int numCircle = 0; numCircle < camCirclesImPts.at(numIm).size(); numCircle++)
        {
            cv::Vec3d intersection = imPlane.intersect(cams[0].backprojectLocal(camCirclesImPts.at(numIm).at(numCircle)));
            circlesObjectPts_.push_back(cv::Point3d(intersection[0], intersection[1], intersection[2]));
        }
        circlesObjectPts.push_back(circlesObjectPts_);
        planes.push_back(Plane(planeRigid));

        // for visualization
        if (numIm == visImIdx)
        {
            pointCloudCircles = new PointCloud(circlesObjectPts_);

            cv::Mat centroidRigid = cv::Mat::eye(cv::Size(4, 4), CV_64F);
            centroidRigid.at<double>(0, 3) = chessboardSqSize * (chessboardDimsX - 1) * 0.5;
            centroidRigid.at<double>(1, 3) = chessboardSqSize * (chessboardDimsY - 1) * 0.5;
            centroidRigid.at<double>(2, 3) = 0;

            cv::Mat planeRigidVis;
            cv::gemm(planeRigid, centroidRigid, 1.0, cv::Mat(), 0.0, planeRigidVis);

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

    // compute projector calibration (inverse R,t gives per-image projector pose in cam coordinate frame?)
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
            projSize.width, projSize.height, 0.015);

        cv::Mat H;

        cv::Mat alignedVirtualProjR, alignedVirtualProjT;
        computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(planes[numIm], cams[0], proj, verticalOffset, alignedVirtualProjR, alignedVirtualProjT);

        Camera transformedProj(
            projK, alignedVirtualProjR, alignedVirtualProjT,
            projSize.width, projSize.height, 0.015);

        computePlaneInducedHomography(planes[numIm], proj, transformedProj, H);

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
            projSize.width, projSize.height, 0.015));

        cv::Mat planeRigid;
        planes[numIm].getRigid(planeRigid);

        cout << planeRigid << endl;

        std::vector<cv::Point3f> transformedChessboardObjectPts;
        for (int i = 0; i < chessboardObjectPts_.size(); i++)
        {
            cv::Vec3d pt(chessboardObjectPts_[i].x, chessboardObjectPts_[i].y, chessboardObjectPts_[i].z);
            cv::Vec3d outPt = Ancillary::Mat44dTimesVec3dHomog(planeRigid, pt);

            transformedChessboardObjectPts.push_back(cv::Point3f(outPt[0], outPt[1], outPt[2]));
        }

        pointCloud = new PointCloud(transformedChessboardObjectPts);
        

        Plane planeProjLocal(planes[numIm].getNormal(), planes[numIm].getDistance());
        planeProjLocal.rigidTransform(cams[1].getRt44());

        cv::Vec3d projPlaneIntersectionLocal = planeProjLocal.intersect(
            cams[1].backprojectLocal(cv::Point2f(projSize.width * 0.5, projSize.height * 0.5)));

        projPlaneIntersection = Ancillary::Mat44dTimesVec3dHomog(cams[1].getRt44Inv(), projPlaneIntersectionLocal);

        cv::Vec3d projCamIntersectionLocal = planes[numIm].intersect(
            cams[0].backprojectLocal(cv::Point2f(camSize.width * 0.5, camSize.height * 0.5)));

        projCamIntersection = Ancillary::Mat44dTimesVec3dHomog(cams[1].getRt44Inv(), projCamIntersectionLocal);

        cv::Mat H, virtualProjR, virtualProjT;
        computeBirdsEyeViewHomography(planes[numIm], cams[1], H, virtualProjR, virtualProjT);
        cams.push_back(Camera(
            projK, virtualProjR, virtualProjT,
            projSize.width, projSize.height, 0.015));

        cv::Mat outIm;
        cv::warpPerspective(visIm, outIm, H, projSize);

        cv::imshow("warped image (w.r.t. virtual projector homography)", outIm);
        cv::waitKey(5);

        cv::Mat alignedVirtualProjR, alignedVirtualProjT;
        computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(planes[numIm], cams[0], cams[1], verticalOffset, alignedVirtualProjR, alignedVirtualProjT);

        cams.push_back(Camera(
            projK, alignedVirtualProjR, alignedVirtualProjT,
            projSize.width, projSize.height, 0.015));

        computePlaneInducedHomography(planes[numIm], cams[1], cams[3], H);

        cv::warpPerspective(visIm, outIm, H, projSize);

        cv::imshow("warped image (w.r.t. final virtual projector homography)", outIm);
        cv::waitKey(5);

        vector<Point3f> imVisPts, imVisColors;
        for (int y = 0; y < projSize.height; y += 3)
        {
            for (int x = 0; x < projSize.width; x += 3)
            {
                cv::Vec3d intersectionLocal = planeProjLocal.intersect(cams[1].backprojectLocal(cv::Point2f(x, y)));
                cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[1].getRt44Inv(), intersectionLocal);
                imVisPts.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));

                cv::Vec3b color = visIm.at<cv::Vec3b>(cv::Point2f(x, y));
                imVisColors.push_back(cv::Point3d(color[2] / 255., color[1] / 255., color[0] / 255.));
            }
        }
        pointCloudImVis = new PointCloud(imVisPts, imVisColors);

        Plane planeProjVirtualLocal(planes[numIm].getNormal(), planes[numIm].getDistance());
        planeProjVirtualLocal.rigidTransform(cams[3].getRt44());

        vector<Point3f> circlesObjectPtsVirtual_;
        for (int numCircle = 0; numCircle < circlesProjPts.at(numIm).size(); numCircle++)
        {
            cv::Vec3d intersectionLocal = planeProjVirtualLocal.intersect(cams[3].backprojectLocal(circlesProjPts.at(numIm).at(numCircle)));
            cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[3].getRt44Inv(), intersectionLocal);
            circlesObjectPtsVirtual_.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));
        }
        pointCloud2Circles = new PointCloud(circlesObjectPtsVirtual_);

        vector<Point3f> imVis2Pts, imVis2Colors;
        for (int y = 0; y < projSize.height; y += 3)
        {
            for (int x = 0; x < projSize.width; x += 3)
            {
                cv::Vec3d intersectionLocal = planeProjVirtualLocal.intersect(cams[3].backprojectLocal(cv::Point2f(x, y)));
                cv::Vec3d intersectionGlobal = Ancillary::Mat44dTimesVec3dHomog(cams[3].getRt44Inv(), intersectionLocal);
                imVis2Pts.push_back(cv::Point3d(intersectionGlobal[0], intersectionGlobal[1], intersectionGlobal[2]));

                cv::Vec3b color = visIm.at<cv::Vec3b>(cv::Point2f(x, y));
                imVis2Colors.push_back(cv::Point3d(color[2] / 255., color[1] / 255., color[0] / 255.));
            }
        }
        pointCloud2ImVis = new PointCloud(imVis2Pts, imVis2Colors);
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
