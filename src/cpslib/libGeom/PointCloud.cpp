 #include "PointCloud.h"

PointCloud::PointCloud()
{
	hasDisplayList_ = false;
}

PointCloud::PointCloud(const PointCloud& points)
{
	points_ = new std::vector<cv::Point3f>(*points.points_);

	hasDisplayList_ = false;
}

PointCloud::PointCloud(std::vector<cv::Point3f>& points)
{
	points_ = new std::vector<cv::Point3f>(points);

	hasDisplayList_ = false;
}

void PointCloud::initPointsDisplayList()
{
	if (hasDisplayList_)
		return;
	
	displayList_ = glGenLists(1);
	glNewList(displayList_, GL_COMPILE);

	for (int x = 0; x < points_->size(); x++)
	{
		cv::Vec3f pt = points_->at(x);

		glBegin(GL_POINTS);
			glVertex3f(pt[0], pt[1], pt[2]);
		glEnd();
	}

	glEndList();

	hasDisplayList_ = true;
}

void PointCloud::display(float pointSize, float r, float g, float b)
{
	if (!hasDisplayList_)
		initPointsDisplayList();

	glColor3f(r, g, b);
	glPointSize(pointSize);
	glCallList(displayList_);

	glFlush();
}
