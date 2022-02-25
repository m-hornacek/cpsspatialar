#include "Plane.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

Plane::Plane()
{
	normal_ = cv::Vec3d(0, 0, -1);
	distance_ = 0;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
}

Plane::Plane(const Plane & plane)
{
	normal_ = cv::Vec3d(plane.normal_);
	distance_ = plane.distance_;

	plane.rigid_.copyTo(rigid_);
}

Plane::Plane(cv::Mat & rigid)
{
	normal_ = cv::Vec3d(0, 0, -1);
	distance_ = 0;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	cv::Vec3d pt = -normal_ * distance_;

	pt = Ancillary::Mat44dTimesVec3dHomog(rigid, pt);

	normal_ = Ancillary::Mat33dTimesVec3d(rigid, normal_);
	distance_ = -normal_.dot(pt);

	rigid.copyTo(rigid_);
}

Plane::Plane(cv::Vec3d normal, float distance)
{
	normal_ = cv::Vec3d(normal);
	distance_ = distance;

	cv::Mat rot = Ancillary::getMinArclengthRotationMat33d(cv::Vec3d(0, 0, -1), normal_);

	Eigen::Matrix3d rotEigen;
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rotEigen(y, x) = rot.at<double>(y, x);

	cv::Vec3d pt = -normal_ * distance_;

	Eigen::Vector3d ptEigen(pt[0], pt[1], pt[2]);
	Eigen::Vector3d tEigen = -rotEigen * ptEigen;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rigid_.at<double>(y, x) = rot.at<double>(y, x);

	for (int i = 0; i < 3; i++)
		rigid_.at<double>(i, 3) = ptEigen[i];

	std::cout << "rigid_: " << rigid_ << std::endl;
}

Plane::Plane(cv::Vec3d normal, cv::Vec3d pt)
{
	normal_ = cv::Vec3d(normal);
	distance_ = -normal.dot(pt);

	cv::Mat rot = Ancillary::getMinArclengthRotationMat33d(cv::Vec3d(0, 0, -1), normal_);

	Eigen::Matrix3d rotEigen;
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rotEigen(y, x) = rot.at<double>(y, x);

	Eigen::Vector3d ptEigen(pt[0], pt[1], pt[2]);
	Eigen::Vector3d tEigen = -rotEigen * ptEigen;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rigid_.at<double>(y, x) = rot.at<double>(y, x);

	for (int i = 0; i < 3; i++)
		rigid_.at<double>(i, 3) = pt[i];

	std::cout << "rigid_: " << rigid_ << std::endl;
}

Plane::Plane(std::vector<cv::Point3f> points, bool ransac)
{
	// copy coordinates to  matrix in Eigen format
	size_t numPts = points.size();
	Eigen::MatrixXd pointsEigen(3, numPts);
	for (size_t i = 0; i < numPts; ++i)
		pointsEigen.col(i) = Eigen::Vector3d(points[i].x, points[i].y, points[i].z);

	// calculate centroid
	Eigen::Vector3d centroidEigen(pointsEigen.row(0).mean(), pointsEigen.row(1).mean(), pointsEigen.row(2).mean());

	if (ransac)
	{
		std::cout << "carrying out RANSAC plane fit" << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		cloud->width = points.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);

		for (int i = 0; i < points.size(); i++)
		{
			cloud->at(i).x = points[i].x;
			cloud->at(i).y = points[i].y;
			cloud->at(i).z = points[i].z;
		}

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(1.0);

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		// todo: ensure normal faces camera
		normal_ = -1 * cv::Vec3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		distance_ = -1 * coefficients->values[3];
	}
	else
	{
		// from https://stackoverflow.com/questions/40589802/eigen-best-fit-of-a-plane-to-n-points

		// subtract centroid
		pointsEigen.row(0).array() -= centroidEigen(0);
		pointsEigen.row(1).array() -= centroidEigen(1);
		pointsEigen.row(2).array() -= centroidEigen(2);

		// we only need the left-singular matrix here
		//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

		auto svd = pointsEigen.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector3d normalEigen = svd.matrixU().rightCols<1>();

		normal_ = -cv::Vec3d(normalEigen[0], normalEigen[1], normalEigen[2]);
		normal_ /= sqrt(normal_.dot(normal_));
		distance_ = -normal_.dot(cv::Vec3d(centroidEigen[0], centroidEigen[1], centroidEigen[2]));
	}

	cv::Mat rot = Ancillary::getMinArclengthRotationMat33d(cv::Vec3d(0, 0, -1), normal_);

	Eigen::Matrix3d rotEigen;
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rotEigen(y, x) = rot.at<double>(y, x);

	Eigen::Vector3d tEigen = -rotEigen * centroidEigen;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rigid_.at<double>(y, x) = rot.at<double>(y, x);

	for (int i = 0; i < 3; i++)
		rigid_.at<double>(i, 3) = centroidEigen[i];
}

void Plane::rigidTransform(cv::Mat & rigid44d)
{
	cv::Vec3d pt = -normal_ * distance_;
	pt = Ancillary::Mat44dTimesVec3dHomog(rigid44d, pt);

	normal_ = Ancillary::Mat33dTimesVec3d(rigid44d, normal_);
	distance_ = -normal_.dot(pt);

	cv::gemm(rigid44d, rigid_, 1.0, cv::Mat(), 0.0, rigid_);
}

cv::Vec3d Plane::intersect(cv::Vec3d ray)
{
	cv::Vec3d pt = -normal_ * distance_; // a point in the plane
	double d = (normal_.dot(pt)) / (ray.dot(normal_)); // distance along ray where ray intersects plane

	return d * ray;
}

double Plane::projDistanceToPlane(cv::Vec3d pt)
{
	return std::abs(normal_.dot(pt) + distance_); // assumes normal has length 1
}

double Plane::projSquaredDistanceToPlane(cv::Vec3d pt)
{
	double dist = projDistanceToPlane(pt);
	return dist * dist;
}

void Plane::display(double radius, float pointSize)
{
	glPointSize(pointSize);
	glColor3f(0.75, 0.75, 0.75);

	// 3D coordinates of plane corner points

	cv::Vec3d ulCanonical = cv::Vec3d(-radius, -radius, 0);
	cv::Vec3d ul3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, ulCanonical);

	cv::Vec3d urCanonical = cv::Vec3d(radius, -radius, 0);
	cv::Vec3d ur3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, urCanonical);

	cv::Vec3d llCanonical = cv::Vec3d(-radius, radius, 0);
	cv::Vec3d ll3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, llCanonical);

	cv::Vec3d lrCanonical = cv::Vec3d(radius, radius, 0);
	cv::Vec3d lr3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, lrCanonical);

	// normal

	//glBegin(GL_LINES);
	//	glVertex3f(centerPoint[0], centerPoint[1], centerPoint[2]);
	//	glVertex3f(radius*normal_[0] + centerPoint[0], radius*normal_[1] + centerPoint[1], radius*normal_[2] + centerPoint[2]);
	//glEnd();


	// plane

	glBegin(GL_LINES);
		glVertex3f(ul3D[0], ul3D[1], ul3D[2]);
		glVertex3f(ur3D[0], ur3D[1], ur3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(ur3D[0], ur3D[1], ur3D[2]);
		glVertex3f(lr3D[0], lr3D[1], lr3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(lr3D[0], lr3D[1], lr3D[2]);
		glVertex3f(ll3D[0], ll3D[1], ll3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(ll3D[0], ll3D[1], ll3D[2]);
		glVertex3f(ul3D[0], ul3D[1], ul3D[2]);
	glEnd();

	glFlush();
}
