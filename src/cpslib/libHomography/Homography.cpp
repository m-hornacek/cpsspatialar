#include "Homography.h"

void Homography::computeBirdsEyeViewHomography(Plane& plane, Camera& proj, cv::Mat& outH, cv::Mat& outR, cv::Mat& outT)
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


void Homography::computeBirdsEyeViewVirtualCam(Plane& plane, Camera& cam, float verticalOffset, cv::Mat& outVirtualCamR, cv::Mat& outVirtualCamT)
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
    Eigen::Vector3d C = centerPt + (verticalOffset /*- distToPlane*/) * n;

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

void Homography::computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(Plane& plane, Camera& cam, Camera& proj, float verticalOffset, cv::Mat& outCamR, cv::Mat& outCamT, cv::Mat& outProjR, cv::Mat& outProjT)
{
    computeBirdsEyeViewVirtualCam(plane, cam, 0, outCamR, outCamT);

    Eigen::Matrix3d virtualCamR;
    cv::cv2eigen(outCamR, virtualCamR);

    Eigen::Vector3d virtualCamT(
        outCamT.at<double>(0, 0),
        outCamT.at<double>(1, 0),
        outCamT.at<double>(2, 0));

    //Eigen::Vector3d virtualCamC = -virtualCamR.inverse() * virtualCamT;

    cv::Vec3d cvCamC = cam.getC();
    Eigen::Vector3d camC(
        cvCamC[0],
        cvCamC[1],
        cvCamC[2]);

    // recompute virtualCamT w.r.t. camC and virtualCamR
    virtualCamT = -virtualCamR * camC;

    outCamT.at<double>(0, 0) = virtualCamT[0];
    outCamT.at<double>(1, 0) = virtualCamT[1];
    outCamT.at<double>(2, 0) = virtualCamT[2];

    cv::Mat cvVirtualProjR, cvVirtualProjT;
    computeBirdsEyeViewVirtualCam(plane, proj, verticalOffset, cvVirtualProjR, cvVirtualProjT);

    Eigen::Matrix3d virtualProjR;
    cv::cv2eigen(cvVirtualProjR, virtualProjR);

    Eigen::Vector3d virtualProjT(
        cvVirtualProjT.at<double>(0, 0),
        cvVirtualProjT.at<double>(1, 0),
        cvVirtualProjT.at<double>(2, 0));

    Eigen::Vector3d virtualProjC = -virtualProjR.inverse() * virtualProjT;

    // recompute virtualProjT w.r.t. virtualProjC and virtualCamR
    virtualProjT = -virtualCamR * virtualProjC;

    outCamR.copyTo(outProjR);

    outProjT = cv::Mat::zeros(3, 1, CV_64F);
    outProjT.at<double>(0, 0) = virtualProjT[0];
    outProjT.at<double>(1, 0) = virtualProjT[1];
    outProjT.at<double>(2, 0) = virtualProjT[2];
}

void Homography::computePlaneInducedHomography(Plane& plane, Camera& cam0, Camera& cam1, cv::Mat& outH)
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