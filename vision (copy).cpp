#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdlib>

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int main() {
	//cv::Mat source = cv::imread("/3314/src/tagged.jpg");
	//cv::Mat source = cv::imread("/3314/src/testing.jpg");	
	cv::Mat source;	
	cv::VideoCapture leftInput(1);
	//cv::VideoCapture rightInput(2);
	double targetX = 0, targetY = 0;
	
	for (;;) {	
		//Capture frame and begin processing timer		
		double t = (double)cv::getTickCount();
		leftInput.read(source);
		//cv::imshow("Source", source);

		//Normalize picture
		cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);	
		cv::Mat output = source.clone(), poss = source.clone();
		cv::Mat hsv, noise;
	
		//HSV threshold
		//cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		//cv::inRange(hsv, cv::Scalar(70,45,195), cv::Scalar(180,255,255), hsv);

		//RGB threshold
		cv::inRange(source, cv::Scalar(0,200,0), cv::Scalar(255,255,255), hsv);

		//Erosion and dilation
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
		cv::erode(hsv, noise, kernel, cv::Point(-1,-1), 1);
		cv::dilate(noise, noise, kernel, cv::Point(-1,-1), 1);
		
		//Output steps 0+1 and 2+3
		//cv::imshow("Thresholded", hsv);
		//cv::imshow("Noise removed", noise);

		//Finding all contours
		std::vector<std::vector<cv::Point> > contours, possible, best;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(noise, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);


		//Filters out bad contours from possible goals
		for (size_t i = 0; i < contours.size(); i++) {
			//Creates two rectangles: regular bounding rect. and min. area. rect.
			std::vector<cv::Point> current = contours[i];
			cv::Rect rect = cv::boundingRect(current);
			cv::RotatedRect rRect = cv::minAreaRect(current);
			cv::Point2f pts[4];
			rRect.points(pts);
		
			//Calculate angle btwn. 0-180deg and vertical offset
			/*Choose the longer edge of the rotated rect to compute the angle*/
			cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
			cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

			cv::Point2f usedEdge = edge1;
			if (cv::norm(edge2) > cv::norm(edge1)) usedEdge = edge2;

			cv::Point2f ref = cv::Vec2f(1,0); /*Horizontal edge*/

			float angle = 180.0f/CV_PI * acos((ref.x*usedEdge.x + ref.y*usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
			float offset = fabs(90.0-angle);

			//Calculate height, width, aspect ratio, and area of min. area. rect.
			double rHeight = cv::norm(pts[1]-pts[0]);
			double rWidth = cv::norm(pts[2]-pts[1]);
			double rRatio = rWidth/rHeight;
			double rArea = rWidth*rHeight;
			//If width is greater than height, use the reciprocal of the ratio
			//This accounts for the fact that the tape's spec dimensions are 2" x 5"
			if (rRatio > 1) {
				rRatio = 1/rRatio;
			}
			
			//Remove targets with an aspect ratio too far from spec 2/5 (0.4)
			if (rRatio < 0.3 || rRatio > 0.5) continue;
			//std::cout << "Ratio: " << rRatio << std::endl;

			//Calculates area and aspect ratio of bounding rect.
			double cArea = cv::contourArea(current);
			double cRatio = (double)(rect.width/rect.height);
			//Calculates how much of the min area rect. the contour fills
			double areaRatio = cArea/rArea;

			//Removes targets where contour doesn't fill enough of the rectangle
			//This means the target isn't shaped like the retroreflective tape
			if (areaRatio < 0.7) continue;
			//std::cout << "Area ratio: " << areaRatio << std::endl;
			
			//Remove targets that are too small (camera noise / robot is too far away)
			if (cArea < 50) continue;
			//std::cout << "Area: " << cArea << std::endl;
			
			//Remove targets that are wider than they are tall
			if (rect.width > rect.height) continue;
			//std::cout << "Width: " << rect.width << " Height: " << rect.height << std::endl;
			
			//Remove targets that are not angled or angled too much
			if (offset < 3 || offset > 20) continue;
			std::cout << "Offset: " << offset << std::endl;

			//Pushes contours that passed to array of possible goals		
			possible.push_back(current);
		}
	
		//
		if (possible.size() > 0) {
			double mostArea = 0;
			double secondMostArea = 0;
			std::vector<cv::Point> firstBest = possible[0];
			std::vector<cv::Point> secondBest = possible[0];

			//Draws mininum area rects. and centers for possible goals
			for (size_t i = 0; i < possible.size(); i++) {
				cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;

				for (unsigned int j = 0; j < 4; ++j) {
					cv::line(poss, pts[j], pts[(j+1)%4], cv::Scalar(0,0,0));
					//double ratio = angle(pts[(j+1)%4], pts[(j+2)%4], pts[j]);
					//std::cout << "Ratio: " << ratio << "; ";
				}
		
				//Commented out but this is distinguish L/R code
				//Centers are red
				cv::circle(poss, center, 2, cv::Scalar(0,0,255));

				for (unsigned int j = 0; j < 4; ++j) {
					//Rights are magenta
					if (angle >= 90) {
						cv::line(poss, pts[j], pts[(j+1)%4], cv::Scalar(255,0,255), 2);
					}
					//Lefts are black
					else {
						cv::line(poss, pts[j], pts[(j+1)%4], cv::Scalar(255,0,0), 2);
					}
				}
				//cv::imshow("Possible", poss);
		
				double area = cv::contourArea(possible[i]);
				//std::cout << "Area: " << area << std::endl;
				if (area > secondMostArea) {
					if (area >= mostArea) {
						secondMostArea = mostArea;
						secondBest = firstBest;
						mostArea = area;
						firstBest = possible[i];
					} else if (offset < mostArea) {
						secondMostArea = area;
						secondBest = possible[i];
					}
				}
				if ((i+1) == possible.size()) {
					best.push_back(firstBest);
					best.push_back(secondBest);
					for (size_t j = 0; j < possible.size(); j++) {
						if (possible[j] == firstBest || possible[j] == secondBest) {
							possible.erase(possible.begin()+j);
						}
					}
				}
			}

			for (size_t i = 0; i < possible.size(); i++) {
				cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
				cv::Point2f pts[4];
				rrect.points(pts);		
				for (unsigned int j = 0; j < 4; ++j) {
					cv::line(output, pts[j], pts[(j+1)%4], cv::Scalar(0,0,0), 2);
				}
				//cv::drawContours(output, possible, i, cv::Scalar(0,0,0), 2);
			}
			cv::Point2f center1, center2;
			for (size_t i = 0; i < best.size(); i++) {
				cv::RotatedRect rrect = cv::minAreaRect(best[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;
				cv::circle(output, center, 2, cv::Scalar(0,0,255));
				//Point 0 is the lowest point, height is distance between 0 and 1, width is distance between 1 and 2
				for (unsigned int j = 0; j < 4; ++j) {
					if (pts[0].x < pts[2].x)  {
						//cv::drawContours(output, best, i, cv::Scalar(255,0,255), 2);
						cv::line(output, pts[j], pts[(j+1)%4], cv::Scalar(255,0,255), 2);
						center1 = center;
					} else {
						//cv::drawContours(output, best, i, cv::Scalar(255,0,0), 2);
						cv::line(output, pts[j], pts[(j+1)%4], cv::Scalar(255,0,0), 2);
						center2 = center;
					}
				}
				//std::cout << "Position: " << center.x << std::endl;
			}
			if (center1.x != 0 && center2.x != 0) {
				targetX = (center1.x+center2.x)/2;
				targetY = (center1.y+center2.y)/2;
			}
		}
		cv::line(output, cv::Point(targetX,0), cv::Point(targetX,output.rows), cv::Scalar(0,0,255), 2);
		//cv::line(output, cv::Point(0,targetY), cv::Point(output.cols,targetY), cv::Scalar(0,255,255), 2);
		cv::imshow("Output", output);
		t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		//std::cout << t*1000 << "ms" << std::endl;
		cv::waitKey(1);
	}
	cv::waitKey();
}
