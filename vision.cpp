#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdlib>

#include <networktables/NetworkTable.h>
#include <stdlib.h>
#include <string.h>

#include "cap_gstreamer.hpp"

//TODO
//Hinting at other targets by pulling from net tables []
//Single target tracking []
//Streaming [x]
//GUI to fix thresholding []
//Run scripts automatically []
//Constants []

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

std::string create_write_pipeline(int width, int height, int framerate,
								  int bitrate, std::string ip, int port)
{

	char buff[500];
	sprintf(buff,
			"appsrc ! "
			"video/x-raw, format=(string)BGR, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
			"videoconvert ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! "
			"udpsink host=%s port=%d",
			width, height, framerate, bitrate, ip.c_str(), port);

	/*sprintf(buff,
	"appsrc ! video/x-raw, format=(string)I420, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host=%s port=%d", width, height, framerate, bitrate, ip.c_str(), port);*/

	std::string pipstring = buff;

	printf("write string: %s\n", pipstring.c_str());
	return pipstring;
}

int main()
{
	bool verbose = true;

	int
		device = 1,
		width = 640,
		height = 480,
		framerate = 15,
		mjpeg = false; //mjpeg is not better than just grabbing a raw image in this case

	//network parameters
	int
		bitrate = 600000,			//kbit/sec over network
		port_stream = 5806,			//destination port for raw image
		port_thresh = 5001;			//destination port for thresholded image
	std::string ip = "192.168.1.3"; //destination ip

	CvVideoWriter_GStreamer mywriter;
	std::string write_pipeline = create_write_pipeline(width, height, framerate,
													   bitrate, ip, port_thresh);
	if (verbose)
	{
		printf("GStreamer write pipeline: %s\n", write_pipeline.c_str());
	}
	mywriter.open(write_pipeline.c_str(),
				  0, framerate, cv::Size(width, height), true);

	long increment = 0;

	//Normalize image
	//cv::Mat source = cv::imread("/3314/src/tagged.jpg");
	//cv::Mat source = cv::imread("/3314/src/testing.jpg");
	cv::Mat source;
	cv::VideoCapture leftInput(1);
	leftInput.set(CV_CAP_PROP_FRAME_WIDTH, width);
	leftInput.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	//cv::VideoCapture rightInput(2);
	double targetX = 0, targetY = 0;
	bool twoTargetsFound = false;
	std::shared_ptr<NetworkTable> myNetTable;
	std::__cxx11::string netTableAddr = "192.168.1.3";
	NetworkTable::SetClientMode();
	NetworkTable::SetDSClientEnabled(false);
	NetworkTable::SetIPAddress(llvm::StringRef(netTableAddr));
	NetworkTable::Initialize();
	myNetTable = NetworkTable::GetTable("SmartDashboard");
	for (;;)
	{
		double t = (double)cv::getTickCount();
		double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
		leftInput.read(source);
		//cv::imshow("Source", source);
		cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);

		cv::Mat output = source.clone(), poss = source.clone();
		cv::Mat hsv, noise;

		//HSV threshold
		cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, cv::Scalar(55, 80, 90), cv::Scalar(255, 255, 255), hsv);
		//cv::inRange(source, cv::Scalar(0,100,0), cv::Scalar(255,255,255), hsv);

		//Erosion and dilation
		//cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		//cv::erode(hsv, noise, kernel, cv::Point(-1, -1), 1);
		//cv::dilate(noise, noise, kernel, cv::Point(-1, -1), 1);

		//cv::imshow("HSV threshold", hsv);
		//cv::imshow("Noise removed", noise);

		t1 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		//Finding all contours
		std::vector<std::vector<cv::Point>> contours, possible, best;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(hsv, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		//Filters out bad contours from possible goals
		for (size_t i = 0; i < contours.size(); i++)
		{
			std::vector<cv::Point> current = contours[i];
			cv::Rect rect = cv::boundingRect(current);
			cv::RotatedRect rRect = cv::minAreaRect(current);
			cv::Point2f pts[4];
			rRect.points(pts);

			/*Choose the longer edge of the rotated rect to compute the angle*/
			cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
			cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

			cv::Point2f usedEdge = edge1;
			if (cv::norm(edge2) > cv::norm(edge1))
				usedEdge = edge2;

			cv::Point2f ref = cv::Vec2f(1, 0); /*Horizontal edge*/

			float angle = 180.0f / CV_PI * acos((ref.x * usedEdge.x + ref.y * usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
			float offset = fabs(90.0 - angle);

			double rHeight = cv::norm(pts[1] - pts[0]);
			double rWidth = cv::norm(pts[2] - pts[1]);
			double rRatio = rWidth / rHeight;
			double rArea = rWidth * rHeight;
			if (rRatio > 1)
			{
				rRatio = 1 / rRatio;
			}
			if (rRatio < 0.2 || rRatio > 0.6)
				continue;
			//std::cout << "Ratio: " << rRatio << std::endl;

			double cArea = cv::contourArea(current); //(double)(rect.width*rect.height);
			double cRatio = (double)(rect.width / rect.height);
			double areaRatio = rArea / cArea;

			//Checks if area is too small/aspect ratio is not close to 2/5
			if (areaRatio < 0.5)
				continue;
			//std::cout << "Area ratio: " << areaRatio << std::endl;
			if (cArea < 25)
				continue;
			//std::cout << "Area: " << cArea << std::endl;
			if (rect.width > rect.height)
				continue;
			//std::cout << "Width: " << rect.width << " Height: " << rect.height << std::endl;
			if (offset < 3 || offset > 20)
				continue;
			//std::cout << "Offset: " << offset << std::endl;
			//if (cRatio < 0.2 || cRatio > 0.6) continue;

			possible.push_back(current);
		}

		t2 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		if (possible.size() > 0)
		{
			double mostArea = 0;
			double secondMostArea = 0;
			std::vector<cv::Point> firstBest = possible[0];
			std::vector<cv::Point> secondBest = possible[0];

			//Draws mininum area rects. and centers for possible goals
			for (size_t i = 0; i < possible.size(); i++)
			{
				cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;

				for (unsigned int j = 0; j < 4; ++j)
				{
					cv::line(poss, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 0, 0));
					//double ratio = angle(pts[(j+1)%4], pts[(j+2)%4], pts[j]);
					//std::cout << "Ratio: " << ratio << "; ";
				}

				//Commented out but this is distinguish L/R code
				//Centers are red
				cv::circle(poss, center, 2, cv::Scalar(0, 0, 255));

				for (unsigned int j = 0; j < 4; ++j)
				{
					//Rights are magenta
					if (pts[0].x < pts[2].x)
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 0, 255), 2);
					}
					//Lefts are black
					else
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
					}
				}
				//cv::imshow("Possible", poss);

				double area = cv::contourArea(possible[i]);
				//std::cout << "Area: " << area << std::endl;
				if (area > secondMostArea)
				{
					if (area >= mostArea)
					{
						secondMostArea = mostArea;
						secondBest = firstBest;
						mostArea = area;
						firstBest = possible[i];
					}
					else if (area < mostArea)
					{
						secondMostArea = area;
						secondBest = possible[i];
					}
				}
			}

			t3 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			best.push_back(firstBest);
			best.push_back(secondBest);
			/*for (size_t j = 0; j < possible.size(); j++)
			{
				if (possible[j] == firstBest || possible[j] == secondBest)
				{
					possible.erase(possible.begin() + j);
				}
			}*/

			for (size_t i = 0; i < possible.size(); i++)
			{
				if (possible[i] != firstBest && possible[i] != secondBest)
				{
					cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
					cv::Point2f pts[4];
					rrect.points(pts);
					for (unsigned int j = 0; j < 4; ++j)
					{
						cv::line(output, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 0, 0), 2);
					}
				}
			}

			t4 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			cv::Point2f center1, center2;
			for (size_t i = 0; i < best.size(); i++)
			{
				cv::RotatedRect rrect = cv::minAreaRect(best[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;
				cv::circle(output, center, 2, cv::Scalar(0, 0, 255));
				//Point 0 is the lowest point, height is distance between 0 and 1, width is distance between 1 and 2
				for (unsigned int j = 0; j < 4; ++j)
				{
					if (pts[0].x < pts[2].x)
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,255), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 0, 255), 2);
						center1 = center;
					}
					else
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,0), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
						center2 = center;
					}
				}
				std::cout << "Position: " << center.x << std::endl;
			}
			if (center1.x != 0 && center2.x != 0)
			{
				targetX = (center1.x + center2.x) / 2;
				targetY = (center1.y + center2.y) / 2;
				twoTargetsFound = true;
			}
			else
			{
				twoTargetsFound = false;
				if (center1.x > 0 || center2.x > 0)
				{
				}
			}
		}
		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, output.rows), cv::Scalar(0, 0, 255), 2);
		//cv::line(output, cv::Point(0,targetY), cv::Point(output.cols,targetY), cv::Scalar(0,255,255), 2);
		t5 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		myNetTable->PutNumber("targetX", targetX - (output.cols / 2));
		myNetTable->PutNumber("targetY", -targetY + (output.rows / 2));
		myNetTable->PutBoolean("twoTargetsFound", twoTargetsFound);
		myNetTable->PutNumber("increment", increment);
		myNetTable->Flush();
		if (increment % 3)
		{
			IplImage outImage = (IplImage)output;
			mywriter.writeFrame(&outImage); //write output image over network
		}
		cv::imshow("Output", output);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		std::cout << t * 1000 << "ms" << std::endl;
		std::cout << t1 * 1000 << "ms " << t2 * 1000 << "ms " << t3 * 1000 << "ms " << t4 * 1000 << "ms " << t5 * 1000 << "ms" << std::endl;
		increment++;
		cv::waitKey(1);
	}
	cv::waitKey();
}
