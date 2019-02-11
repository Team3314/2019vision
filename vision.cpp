#include "vision.hpp"

//TODO
//Hinting at other targets by pulling from net tables []
//Single target tracking [x]
//Streaming [x]
//GUI to fix thresholding []
//Run scripts automatically [x]
//Constants [x]
//Confidence levels []

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

void flash_good_settings(int device)
{
	char setting_script[100];
	sprintf(setting_script, "/3314/src/good_settings.sh %d", device);
	system(setting_script);
}

void flash_bad_settings(int device)
{
	char setting_script[100];
	sprintf(setting_script, "/3314/src/bad_settings.sh %d", device);
	system(setting_script);
}

void setVideoCaps(cv::VideoCapture& input) {
	input.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	input.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
}

class TargetTracker
{
	cv::VideoCapture input;

	public: 
	long frame;
	int device;
	cv::Mat source;
	cv::Mat output;

	double baseOffset = 0;
	double targetX=0;
	double targetY=0;
	double targetAngle = 0;
	bool twoTargetsFound = false;

	TargetTracker(int Device, double BaseOffset) 
	{
		device = Device;
		baseOffset = BaseOffset;
		input = new cv::VideoCapture(device);
		setVideoCaps(input)
	}

	void capture() 
	{
		frame++;
		if (frame == 50)
		{
			flash_bad_settings(device);
		}
		else if (frame == 0 || frame == 100)
		{
			flash_good_settings(device);
		}

		input.read(source);
	}

	void analyze(cv::Mat& source) 
	{
		double t = (double)cv::getTickCount();
		double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
		//cv::imshow("Source", source);
		cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);

		//cv::Mat output = source.clone(), poss = source.clone();
		cv::Mat poss = source.clone();
		cv::Mat hsv;
		//cv::Mat noise;

		output = source.clone(); 

		//HSV threshold
		cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, MIN_HSV, MAX_HSV, hsv);

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
			if (rRatio < MIN_ASPECT_RATIO || rRatio > MAX_ASPECT_RATIO)
				continue;
			//std::cout << "Ratio: " << rRatio << std::endl;

			double cArea = cv::contourArea(current); //(double)(rect.width*rect.height);
			double cRatio = (double)(rect.width / rect.height);
			double areaRatio = rArea / cArea;

			//Checks if area is too small/aspect ratio is not close to 2/5
			if (areaRatio < MIN_AREA_RATIO)
				continue;
			//std::cout << "Area ratio: " << areaRatio << std::endl;
			if (cArea < MIN_AREA)
				continue;
			//std::cout << "Area: " << cArea << std::endl;
			if (rect.width > rect.height)
				continue;
			//std::cout << "Width: " << rect.width << " Height: " << rect.height << std::endl;
			if (offset < MIN_OFFSET || offset > MAX_OFFSET)
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
					cv::line(poss, pts[j], pts[(j + 1) % 4], BLACK);
					//double ratio = angle(pts[(j+1)%4], pts[(j+2)%4], pts[j]);
					//std::cout << "Ratio: " << ratio << "; ";
				}

				//Commented out but this is distinguish L/R code
				//Centers are red
				cv::circle(poss, center, 2, RED);

				for (unsigned int j = 0; j < 4; ++j)
				{
					//Rights are magenta
					if (pts[0].x < pts[2].x)
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], PINK, 2);
					}
					//Lefts are black
					else
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], BLUE, 2);
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
			if (firstBest != secondBest)
			{
				best.push_back(secondBest);
			}
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
						cv::line(output, pts[j], pts[(j + 1) % 4], BLACK, 2);
					}
				}
			}

			t4 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			cv::Point2f rightCenter, leftCenter;
			for (size_t i = 0; i < best.size(); i++)
			{
				cv::RotatedRect rrect = cv::minAreaRect(best[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;
				cv::circle(output, center, 2, RED);
				//Point 0 is the lowest point, height is distance between 0 and 1, width is distance between 1 and 2
				for (unsigned int j = 0; j < 4; ++j)
				{
					if (pts[0].x < pts[2].x)
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,255), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], PINK, 2);
						rightCenter = center;
					}
					else
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,0), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], BLUE, 2);
						leftCenter = center;
					}
				}
				//std::cout << "Position: " << center.x << std::endl;
			}
			if (leftCenter.x != 0 && rightCenter.x != 0)
			{
				targetX = (rightCenter.x + leftCenter.x) / 2;
				targetY = (rightCenter.y + leftCenter.y) / 2;
				distance = (rightCenter.x - leftCenter.x) / 2;
				twoTargetsFound = true;
			}
			else
			{
				twoTargetsFound = false;
				if (rightCenter.x != 0)
				{
					targetX = rightCenter.x - distance;
				}
				if (leftCenter.x != 0)
				{
					targetX = leftCenter.x + distance;
				}
			}
		}

		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, HEIGHT), cv::Scalar(0, 0, 255), 2);

		targetX = targetX - (WIDTH / 2);
		targetY = -targetY + (HEIGHT / 2);
		targetAngle = targetX * HORZ_DEGREES_PER_PIXEL + baseOffset; // Could be a more complex calc, we'll see if we need it 

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		std::cout << t * 1000 << "ms" << std::endl;
		std::cout << t1 * 1000 << "ms " << t2 * 1000 << "ms " << t3 * 1000 << "ms " << t4 * 1000 << "ms " /*<< t5 * 1000 << "ms"*/ << std::endl;

	}
}

int main()
{

	bool verbose = true;

	CvVideoWriter_GStreamer mywriter;
	std::string write_pipeline = create_write_pipeline(WIDTH, HEIGHT, FRAMERATE,
													   BITRATE, IP, PORT);
	if (verbose)
	{
		printf("GStreamer write pipeline: %s\n", write_pipeline.c_str());
	}
	mywriter.open(write_pipeline.c_str(),
				  0, FRAMERATE, cv::Size(WIDTH, HEIGHT), true);

	long increment = 0;

	TargetTracker leftTracker(1);
	TargetTracker rightTracker(2);
	leftTracker.baseOffset = CAMERA_BASE_ANGLE;
	rightTracker.baseOffset = -CAMERA_BASE_ANGLE;

	double distance = 0;
	double ofs = 0;
	double angleToTarget=0;

	std::shared_ptr<NetworkTable> myNetTable;
	NetworkTable::SetClientMode();
	NetworkTable::SetDSClientEnabled(false);
	NetworkTable::SetIPAddress(llvm::StringRef(IP));
	NetworkTable::Initialize();
	myNetTable = NetworkTable::GetTable("SmartDashboard");

	for (;;)
	{
		std::cout << "Increment: " << increment << std::endl;

		leftInput.capture();
		rightInput.capture();

		leftTracker.analyze();
		rightTracker.analyze();

		distance = -1;
		if(leftTracker.twoTargetsFound && righTracker.twoTargetsFound)
		{
			double tanLeft = tan(leftTracker.targetAngle);
			double tanRight = tan(rightTracker.targetAngle);
			distance = CAMERA_SEPARATION/(tanLeft+tanRight);
			ofs = tanLeft*distance - CAMERA_SEPARATION/2.0;
			angleToTarget = atan(ofs/dist); 
		}

		//t5 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		myNetTable->PutNumber("Left targetX", leftInput.targetX);
		myNetTable->PutNumber("Left targetY", leftInput.targetY);
		myNetTable->PutBoolean("Left twoTargetsFound", leftInput.twoTargetsFound);
		myNetTable->PutNumber("Right targetX", rightInput.targetX);
		myNetTable->PutNumber("Right targetY", rightInput.targetY);
		myNetTable->PutBoolean("Right twoTargetsFound", rightInput.twoTargetsFound);
		myNetTable->PutNumber("Distance", distance);
		myNetTable->PutNumber("Offset", ofs);
		myNetTable->PutNumber("Angle To Target", angleToTarget);
		myNetTable->PutNumber("increment", increment);
		myNetTable->Flush();
		if (increment % 3)
		{
			IplImage outImage = (IplImage)leftTracker.output;
			mywriter.writeFrame(&outImage); //write output image over network
		}
		cv::imshow("Left Output", leftTracker.output);
		cv::imshow("Right Output", rightTracker.output);

		increment++;
		cv::waitKey(1);
	}
	cv::waitKey();
}
