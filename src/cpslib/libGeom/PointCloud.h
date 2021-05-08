#ifndef POINTCLOUD
#define POINTCLOUD
#include "headers.h"

class PointCloud
{
public:
	PointCloud(); // default constructor
	PointCloud(const PointCloud& points);
	PointCloud(std::vector<cv::Point3f>& points);
	
	std::vector<cv::Point3f> getPoints() { return *points_; };
	
	void display(float pointSize, float r, float g, float b);

	~PointCloud(void) { };
private:
	void initPointsDisplayList();

	std::vector<cv::Point3f> * points_;

	bool hasDisplayList_;
	GLuint displayList_;
};

#endif
 