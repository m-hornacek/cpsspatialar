#ifndef HEADERS
#define HEADERS

#define PARALLEL

#define NANVAL -160000 // value for blank areas in depth maps obtained from OpenCV disparity maps
#define PI 3.14159
#define CPU_COUNT 4 // used as is in nested parallelization, squared otherwise

#ifdef PARALLEL
#include "omp.h" // OpenMP
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>
#include <algorithm>
#include <utility>
#include <ctime>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include "Eigen/Geometry"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace Ancillary
{
	inline std::string mat33dToString(cv::Mat mat33d)
	{
		std::ostringstream o;

		o << mat33d.at<double>(0,0) << ", " << mat33d.at<double>(0,1) << ", " << mat33d.at<double>(0,2) << std::endl;
		o << mat33d.at<double>(1,0) << ", " << mat33d.at<double>(1,1) << ", " << mat33d.at<double>(1,2) << std::endl;
		o << mat33d.at<double>(2,0) << ", " << mat33d.at<double>(2,1) << ", " << mat33d.at<double>(2,2) << std::endl;

		return o.str();
	}
	
	inline std::string mat34dToString(cv::Mat mat34d)
	{
		std::ostringstream o;

		o << mat34d.at<double>(0,0) << ", " << mat34d.at<double>(0,1) << ", " << mat34d.at<double>(0,2) << ", " << mat34d.at<double>(0,3) << std::endl;
		o << mat34d.at<double>(1,0) << ", " << mat34d.at<double>(1,1) << ", " << mat34d.at<double>(1,2) << ", " << mat34d.at<double>(1,3) << std::endl;
		o << mat34d.at<double>(2,0) << ", " << mat34d.at<double>(2,1) << ", " << mat34d.at<double>(2,2) << ", " << mat34d.at<double>(2,3) << std::endl;

		return o.str();
	}

	inline std::string mat44dToString(cv::Mat mat44d)
	{
		std::ostringstream o;

		o << mat44d.at<double>(0,0) << ", " << mat44d.at<double>(0,1) << ", " << mat44d.at<double>(0,2) << ", " << mat44d.at<double>(0,3) << std::endl;
		o << mat44d.at<double>(1,0) << ", " << mat44d.at<double>(1,1) << ", " << mat44d.at<double>(1,2) << ", " << mat44d.at<double>(1,3) << std::endl;
		o << mat44d.at<double>(2,0) << ", " << mat44d.at<double>(2,1) << ", " << mat44d.at<double>(2,2) << ", " << mat44d.at<double>(2,3) << std::endl;
		o << mat44d.at<double>(3,0) << ", " << mat44d.at<double>(3,1) << ", " << mat44d.at<double>(3,2) << ", " << mat44d.at<double>(3,3) << std::endl;

		return o.str();
	}

	inline int closestGreaterOrEqaulPowerOf2(int num)
	{
		int ret = 1;

		while (ret <= num)
			ret *= 2;

		if (ret < num)
			ret *= 2;

		return ret;
	}

	inline cv::Vec3f convertDepthToXYZ(float f, float x, float y, float Z, float px, float py)
	{
		return cv::Vec3f((x - px) / f * Z, (y - py) / f * Z, Z);
	}

	inline cv::Mat convertDepthMap8UToXYZMap(float f, cv::Mat depthMap8U, float px = -1, float py = -1)
	{
		cv::Mat XYZMap(depthMap8U.size(), CV_32FC3);

		if (px == -1)
			px = 0.5 * XYZMap.cols;

		if (py == -1)
			py = 0.5 * XYZMap.rows;

		for (int y = 0; y < XYZMap.rows; y++)
		{
			for (int x = 0; x < XYZMap.cols; x++)
			{
				float Z = depthMap8U.at<uchar>(cv::Point(x, y));
				float X = (x - px) / f * Z;
				float Y = (y - py) / f * Z;

				XYZMap.at<cv::Vec3f>(cv::Point(x, y)) = cv::Vec3f(X, Y, Z);
			}
		}

		return XYZMap;
	}

	inline cv::Mat convertDepthMap32FToXYZMap(float f, cv::Mat depthMap32F, float px = -1, float py = -1)
	{
		cv::Mat XYZMap(depthMap32F.size(), CV_32FC3);

		px = (px == -1) ? 0.5 * XYZMap.cols : px;
		py = (py == -1) ? 0.5 * XYZMap.rows : py;

		for (int y = 0; y < XYZMap.rows; y++)
		{
			for (int x = 0; x < XYZMap.cols; x++)
			{
				float Z = depthMap32F.at<float>(cv::Point(x, y));
				float X = (x - px) / f * Z;
				float Y = (y - py) / f * Z;

				XYZMap.at<cv::Vec3f>(cv::Point(x, y)) = cv::Vec3f(X, Y, Z);
			}
		}

		return XYZMap;
	}

	// assumes 4th component of vec is 1
	inline cv::Vec3d Mat44dTimesVec3dHomog(cv::Mat& mat44d, cv::Vec3d& vec)
	{
		cv::Vec3d ret;

		ret[0] = mat44d.at<double>(0,0) * vec[0] + mat44d.at<double>(0,1) * vec[1] + mat44d.at<double>(0,2) * vec[2] + mat44d.at<double>(0,3);
		ret[1] = mat44d.at<double>(1,0) * vec[0] + mat44d.at<double>(1,1) * vec[1] + mat44d.at<double>(1,2) * vec[2] + mat44d.at<double>(1,3);
		ret[2] = mat44d.at<double>(2,0) * vec[0] + mat44d.at<double>(2,1) * vec[1] + mat44d.at<double>(2,2) * vec[2] + mat44d.at<double>(2,3);

		double s = mat44d.at<double>(3,0) * vec[0] + mat44d.at<double>(3,1) * vec[1] + mat44d.at<double>(3,2) * vec[2] + mat44d.at<double>(3,3);

		ret[0] /= s;
		ret[1] /= s;
		ret[2] /= s;

		return ret;
	}

	inline cv::Vec3d Mat44dTimesVec4dHomog(cv::Mat& mat44d, cv::Vec4d& vec)
	{
		cv::Vec3d ret;

		ret[0] = mat44d.at<double>(0,0) * vec[0] + mat44d.at<double>(0,1) * vec[1] + mat44d.at<double>(0,2) * vec[2] + mat44d.at<double>(0,3) * vec[3];
		ret[1] = mat44d.at<double>(1,0) * vec[0] + mat44d.at<double>(1,1) * vec[1] + mat44d.at<double>(1,2) * vec[2] + mat44d.at<double>(1,3) * vec[3];
		ret[2] = mat44d.at<double>(2,0) * vec[0] + mat44d.at<double>(2,1) * vec[1] + mat44d.at<double>(2,2) * vec[2] + mat44d.at<double>(2,3) * vec[3];

		double s = mat44d.at<double>(3,0) * vec[0] + mat44d.at<double>(3,1) * vec[1] + mat44d.at<double>(3,2) * vec[2] + mat44d.at<double>(3,3) * vec[3];

		ret[0] /= s;
		ret[1] /= s;
		ret[2] /= s;

		return ret;
	}

	// assumes 4th component of vec is 1
	inline cv::Vec3f Mat44dTimesVec3dHomog(cv::Mat& mat44d, cv::Vec3f& vec)
	{
		cv::Vec3d retDouble;

		retDouble[0] = mat44d.at<double>(0,0) * vec[0] + mat44d.at<double>(0,1) * vec[1] + mat44d.at<double>(0,2) * vec[2] + mat44d.at<double>(0,3);
		retDouble[1] = mat44d.at<double>(1,0) * vec[0] + mat44d.at<double>(1,1) * vec[1] + mat44d.at<double>(1,2) * vec[2] + mat44d.at<double>(1,3);
		retDouble[2] = mat44d.at<double>(2,0) * vec[0] + mat44d.at<double>(2,1) * vec[1] + mat44d.at<double>(2,2) * vec[2] + mat44d.at<double>(2,3);

		double s = mat44d.at<double>(3,0) * vec[0] + mat44d.at<double>(3,1) * vec[1] + mat44d.at<double>(3,2) * vec[2] + mat44d.at<double>(3,3);

		retDouble[0] /= s;
		retDouble[1] /= s;
		retDouble[2] /= s;

		return cv::Vec3f((float)(retDouble[0]), (float)(retDouble[1]), (float)(retDouble[2]));
	}

	inline cv::Vec3d Mat33dTimesVec3d(cv::Mat& mat33d, cv::Vec3d& vec)
	{
		cv::Vec3d ret;

		ret[0] = mat33d.at<double>(0,0) * vec[0] + mat33d.at<double>(0,1) * vec[1] + mat33d.at<double>(0,2) * vec[2];
		ret[1] = mat33d.at<double>(1,0) * vec[0] + mat33d.at<double>(1,1) * vec[1] + mat33d.at<double>(1,2) * vec[2];
		ret[2] = mat33d.at<double>(2,0) * vec[0] + mat33d.at<double>(2,1) * vec[1] + mat33d.at<double>(2,2) * vec[2];

		return ret;
	}

	// 3D rotation from srcNormal to dstNormal such that arc length is minimized
	inline cv::Mat getMinArclengthRotationMat33d(cv::Vec3d& srcNormal, cv::Vec3d& dstNormal)
	{
		cv::Vec3d dstNormalCorrected = (dstNormal[2] == 0) ? cv::Vec3d(0, 0, -1) : dstNormal;

		Eigen::Vector3f vec0(srcNormal[0], srcNormal[1], srcNormal[2]);
		Eigen::Vector3f vec1(dstNormalCorrected[0], dstNormalCorrected[1], dstNormalCorrected[2]);

		Eigen::Quaternionf q;
		q.setFromTwoVectors(vec0, vec1);
		Eigen::Matrix3f mat = q.toRotationMatrix();

		cv::Mat ret(3, 3, CV_64F);
		ret.at<double>(0, 0) = mat(0, 0);
		ret.at<double>(1, 0) = mat(1, 0);
		ret.at<double>(2, 0) = mat(2, 0);
		ret.at<double>(0, 1) = mat(0, 1);
		ret.at<double>(1, 1) = mat(1, 1);
		ret.at<double>(2, 1) = mat(2, 1);
		ret.at<double>(0, 2) = mat(0, 2);
		ret.at<double>(1, 2) = mat(1, 2);
		ret.at<double>(2, 2) = mat(2, 2);

		return ret;
	}

	inline cv::Mat getMinArclengthRotationMat44d(cv::Vec3d& srcNormal, cv::Vec3d& dstNormal)
	{
		cv::Mat rot33d = getMinArclengthRotationMat33d(srcNormal, dstNormal);

		cv::Mat ret(4, 4, CV_64F);

		ret.at<double>(0, 0) = rot33d.at<double>(0, 0);
		ret.at<double>(1, 0) = rot33d.at<double>(1, 0);
		ret.at<double>(2, 0) = rot33d.at<double>(2, 0);
		ret.at<double>(3, 0) = 0;
		ret.at<double>(0, 1) = rot33d.at<double>(0, 1);
		ret.at<double>(1, 1) = rot33d.at<double>(1, 1);
		ret.at<double>(2, 1) = rot33d.at<double>(2, 1);
		ret.at<double>(3, 1) = 0;
		ret.at<double>(0, 2) = rot33d.at<double>(0, 2);
		ret.at<double>(1, 2) = rot33d.at<double>(1, 2);
		ret.at<double>(2, 2) = rot33d.at<double>(2, 2);
		ret.at<double>(3, 2) = 0;
		ret.at<double>(0, 3) = 0;
		ret.at<double>(1, 3) = 0;
		ret.at<double>(2, 3) = 0;
		ret.at<double>(3, 3) = 1;

		return ret;
	}

	// rigid body transformation relating oriented points (from src to dst)
	inline cv::Mat getRigid(cv::Vec3d& srcCenterPoint, cv::Vec3d& dstCenterPoint, cv::Vec3d& srcNormal, cv::Vec3d& dstNormal)
	{
		cv::Mat ret, temp;

		cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
		cv::Mat B = cv::Mat::zeros(4, 4, CV_64F);
		cv::Mat C = cv::Mat::zeros(4, 4, CV_64F);

		A.at<double>(0,0) = 1;
		A.at<double>(0,3) = -srcCenterPoint[0];
		A.at<double>(1,1) = 1;
		A.at<double>(1,3) = -srcCenterPoint[1];
		A.at<double>(2,2) = 1;
		A.at<double>(2,3) = -srcCenterPoint[2];
		A.at<double>(3,3) = 1;

		cv::Mat rot = getMinArclengthRotationMat33d(srcNormal, dstNormal);
		cv::Vec3d transl = dstCenterPoint - srcCenterPoint;

		B.at<double>(0,0) = rot.at<double>(0,0);
		B.at<double>(0,1) = rot.at<double>(0,1);
		B.at<double>(0,2) = rot.at<double>(0,2);
		B.at<double>(0,3) = transl[0];
		B.at<double>(1,0) = rot.at<double>(1,0);
		B.at<double>(1,1) = rot.at<double>(1,1);
		B.at<double>(1,2) = rot.at<double>(1,2);
		B.at<double>(1,3) = transl[1];
		B.at<double>(2,0) = rot.at<double>(2,0);
		B.at<double>(2,1) = rot.at<double>(2,1);
		B.at<double>(2,2) = rot.at<double>(2,2);
		B.at<double>(2,3) = transl[2];
		B.at<double>(3,3) = 1;

		C.at<double>(0,0) = 1;
		C.at<double>(0,3) = srcCenterPoint[0];
		C.at<double>(1,1) = 1;
		C.at<double>(1,3) = srcCenterPoint[1];
		C.at<double>(2,2) = 1;
		C.at<double>(2,3) = srcCenterPoint[2];
		C.at<double>(3,3) = 1;

		// ret = C * B * A
		cv::gemm(B, A, 1.0, cv::Mat(), 0.0, temp);
		cv::gemm(C, temp, 1.0, cv::Mat(), 0.0, ret);

		return ret;
	}

	// transl is the non-rigid translation
	inline cv::Mat getRigid(cv::Vec3d& srcOrigin, const cv::Mat& rot, const cv::Vec3d& transl)
	{
		cv::Mat ret, temp;

		cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
		cv::Mat B = cv::Mat::zeros(4, 4, CV_64F);
		cv::Mat C = cv::Mat::zeros(4, 4, CV_64F);

		A.at<double>(0,0) = 1;
		A.at<double>(0,3) = -srcOrigin[0];
		A.at<double>(1,1) = 1;
		A.at<double>(1,3) = -srcOrigin[1];
		A.at<double>(2,2) = 1;
		A.at<double>(2,3) = -srcOrigin[2];
		A.at<double>(3,3) = 1;

		B.at<double>(0,0) = rot.at<double>(0,0);
		B.at<double>(0,1) = rot.at<double>(0,1);
		B.at<double>(0,2) = rot.at<double>(0,2);
		B.at<double>(0,3) = transl[0];
		B.at<double>(1,0) = rot.at<double>(1,0);
		B.at<double>(1,1) = rot.at<double>(1,1);
		B.at<double>(1,2) = rot.at<double>(1,2);
		B.at<double>(1,3) = transl[1];
		B.at<double>(2,0) = rot.at<double>(2,0);
		B.at<double>(2,1) = rot.at<double>(2,1);
		B.at<double>(2,2) = rot.at<double>(2,2);
		B.at<double>(2,3) = transl[2];
		B.at<double>(3,3) = 1;

		C.at<double>(0,0) = 1;
		C.at<double>(0,3) = srcOrigin[0];
		C.at<double>(1,1) = 1;
		C.at<double>(1,3) = srcOrigin[1];
		C.at<double>(2,2) = 1;
		C.at<double>(2,3) = srcOrigin[2];
		C.at<double>(3,3) = 1;

		cv::gemm(B, A, 1.0, cv::Mat(), 0.0, temp);
		cv::gemm(C, temp, 1.0, cv::Mat(), 0.0, ret);

		return ret;
	}

	inline cv::Vec2d projectPoint(cv::Mat projMatrix34d, cv::Vec3d pt, bool roundToInt = false)
	{
		cv::Vec2d ret;
	
		ret[0]   = projMatrix34d.at<double>(0,0) * pt[0] + projMatrix34d.at<double>(0,1) * pt[1] + projMatrix34d.at<double>(0,2) * pt[2];
		ret[1]   = projMatrix34d.at<double>(1,0) * pt[0] + projMatrix34d.at<double>(1,1) * pt[1] + projMatrix34d.at<double>(1,2) * pt[2];
		double s = projMatrix34d.at<double>(2,0) * pt[0] + projMatrix34d.at<double>(2,1) * pt[1] + projMatrix34d.at<double>(2,2) * pt[2];

		ret[0] /= s;
		ret[1] /= s;

		if (roundToInt)
		{
			int intX = int(ret[0]);
			int intY = int(ret[1]);

			ret[0] = (ret[0] - intX < 0.5) ? intX : intX + 1;
			ret[1] = (ret[1] - intY < 0.5) ? intY : intY + 1;
		}

		return ret;
	}

	// column-major flattening (as expected by glMultMatrix)
	inline double* flattenMat33d(cv::Mat& mat33d)
	{
		double *ret = new double[16];
		ret[0] = mat33d.at<double>(0, 0);
		ret[1] = mat33d.at<double>(1, 0);
		ret[2] = mat33d.at<double>(2, 0);
		ret[3] = 0;
		ret[4] = mat33d.at<double>(0, 1);
		ret[5] = mat33d.at<double>(1, 1);
		ret[6] = mat33d.at<double>(2, 1);
		ret[7] = 0;
		ret[8] = mat33d.at<double>(0, 2);
		ret[9] = mat33d.at<double>(1, 2);
		ret[10] = mat33d.at<double>(2, 2);
		ret[11] = 0;
		ret[12] = 0;
		ret[13] = 0;
		ret[14] = 0;
		ret[15] = 1;

		return ret;
	}

	// column-major flattening (as expected by glMultMatrix)
	inline double* flattenMat44d(cv::Mat& mat44d)
	{
		//double *ret = new double[16];
		//ret[0] = mat44d.at<double>(0, 0);
		//ret[1] = mat44d.at<double>(1, 0);
		//ret[2] = mat44d.at<double>(2, 0);
		//ret[3] = 0;
		//ret[4] = mat44d.at<double>(0, 1);
		//ret[5] = mat44d.at<double>(1, 1);
		//ret[6] = mat44d.at<double>(2, 1);
		//ret[7] = 0;
		//ret[8] = mat44d.at<double>(0, 2);
		//ret[9] = mat44d.at<double>(1, 2);
		//ret[10] = mat44d.at<double>(2, 2);
		//ret[11] = 0;
		//ret[12] = 0;
		//ret[13] = 0;
		//ret[14] = 0;
		//ret[15] = 1;

		double *ret = new double[16];
		ret[0] = mat44d.at<double>(0, 0);
		ret[1] = mat44d.at<double>(1, 0);
		ret[2] = mat44d.at<double>(2, 0);
		ret[3] = mat44d.at<double>(3, 0);
		ret[4] = mat44d.at<double>(0, 1);
		ret[5] = mat44d.at<double>(1, 1);
		ret[6] = mat44d.at<double>(2, 1);
		ret[7] = mat44d.at<double>(3, 1);
		ret[8] = mat44d.at<double>(0, 2);
		ret[9] = mat44d.at<double>(1, 2);
		ret[10] = mat44d.at<double>(2, 2);
		ret[11] = mat44d.at<double>(3, 2);
		ret[12] = mat44d.at<double>(0, 3);
		ret[13] = mat44d.at<double>(1, 3);
		ret[14] = mat44d.at<double>(2, 3);
		ret[15] = mat44d.at<double>(3, 3);

		return ret;
	}

	// todo: allow some kind of epsilon?
	inline bool areRigidBodiesEqual(cv::Mat m0, cv::Mat m1)
	{
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				if (m0.at<double>(x, y) != m1.at<double>(x, y))
					return false;
			}
		}

		return true;
	}
}

#endif
