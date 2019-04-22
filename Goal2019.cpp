#include "Goal2019.hpp"

	Goal2019::Goal2019(std::vector<cv::Point> Contour)
	{
		contour = Contour;
		rect = cv::boundingRect(contour);
		rRect = cv::minAreaRect(contour);
		rRect.points(pts);
		center = rRect.center;

		angle = TargetTracker::rrectAngleEdge(TargetTracker::findLongEdge(pts));
		offset = TargetTracker::rrectAngleOffset(angle);

		rHeight = cv::norm(TargetTracker::findLongEdge(pts));
		rWidth = cv::norm(TargetTracker::findShortEdge(pts));

		/*angle = rrectAnglePts(pts);
		offset = rrectAngleOffset(angle);

		rHeight = cv::norm(pts[1] - pts[0]);
		rWidth = cv::norm(pts[2] - pts[1]);*/

		rArea = rWidth * rHeight;
		rRatio = rWidth / rHeight;
		if (rRatio > 1)
		{
			rRatio = 1 / rRatio;
		}

		cArea = cv::contourArea(contour);
		cRatio = (double)(rect.width / rect.height);
		areaRatio = rArea / cArea;

		if (angle > 90)
		{
			isLeft = false;
			isRight = true;
		}
		else
		{
			isLeft = true;
			isRight = false;
		}

		partner = -1;
	}
