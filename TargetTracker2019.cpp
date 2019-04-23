#include "TargetTracker2019.hpp"

void TargetTracker2019::analyze()
{
	targetsFound = 0, centeredTargetX = 0, centeredTargetY = 0, targetAngle = 0, hasLeft = false, hasRight = false;

	double lastGoodLRSeparation;

	cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);
	output = source.clone();

	//HSV threshold
	cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
	cv::inRange(hsv, minHSV, maxHSV, hsv);

	//Finding all contours
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(hsv, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
	if (verbose)
	{
		std::cout << "\nTotal contours: " << contours.size() << std::endl;
	}

	//Filters out bad contours from possible goals
	for (size_t i = 0; i < contours.size(); i++)
	{
		// TODO: this should probably scale with camera resolution
		if (cv::contourArea(contours[i]) > MIN_AREA * 8)
		{
			//if (verbose)
			//{
			//	std::cout << "Failed min area" << std::endl;
			//}
			continue;
		}
		contours.erase(contours.begin() + i);
	}
	if (verbose)
	{
		std::cout << "\nFirst cull contours: " << contours.size() << std::endl;
	}

	// The hatch mechanism causes reflections that show up as 
	// high interest contours. These can be dealt with by ignoring
	// contours which exist only above the top fifth of the screen.
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::RotatedRect rRect1 = cv::minAreaRect(contours[i]);
		cv::Point2f pts1[4];
		rRect1.points(pts1);
		if (pts1[0].y < (camInfo->ImageWidth / 5))
		{
			contours.erase(contours.begin() + i);
			i--;
		}
	}

	//Take min and max x-pos of two contours in question
	//If any x-pos in between, passes
	//If passes, convex hull around 2 and erase old two contours
	// (maybe loop this while changes are made (with some limit))
	MergeTargets();
	//Running twice to remove outliers that got through
	MergeTargets(); 

	// Draw the current contours of interest
	for (int i = 0; i < contours.size(); i++)
		cv::drawContours(output, contours, i, WHITE, 2);

	//Filters out bad contours from possible goals
	for (size_t i = 0; i < contours.size(); i++)
	{
		Goal2019 goal(contours[i]);

		if (goal.areaRatio < MIN_AREA_RATIO)
		{
			if (verbose)
			{
				std::cout << "Failed min area ratio" << std::endl;
			}
			continue;
		}
		if (goal.cArea < MIN_AREA * 8)
		{
			if (verbose)
			{
				std::cout << "Failed min area" << std::endl;
			}
			continue;
		}
		if (goal.rect.width > goal.rect.height)
		{
			if (verbose)
			{
				std::cout << "Failed vertical check" << std::endl;
			}
			continue;
		}
		if (goal.offset < MIN_OFFSET || goal.offset > MAX_OFFSET)
		{
			if (verbose)
			{
				std::cout << "Failed correct angle" << std::endl;
			}
			continue;
		}

		possible.push_back(goal);
	}

	if (verbose)
	{
		std::cout << "Total possibles: " << possible.size() << std::endl;
	}

	double bestDistBetween = 10000;
	if (possible.size() > 0)
	{
		//match left and right pairs
		int size = possible.size();
		for (size_t i = 0; i < size; i++)
		{
			double min = possible[i].pts[2].y;
			double max = possible[i].pts[0].y;
			if (possible[i].partner == -1)
			{
				possible[i].partner = i;
			}
			for (size_t j = i + 1; j < size; j++)
			{
				double min2 = possible[j].pts[2].y;
				double max2 = possible[j].pts[0].y;
				if (possible[j].partner == -1)
				{
					possible[j].partner = j;
				}
				if (possible[i].isLeft == possible[j].isLeft || possible[i].isRight == possible[j].isRight)
					continue;
				if (possible[i].isLeft && possible[j].isRight && possible[i].center.x > possible[j].center.x)
					continue;
				if (possible[i].isRight && possible[j].isLeft && possible[i].center.x < possible[j].center.x)
					continue;
				//if (isBetween(possible[i].pts[0].x, possible[i].pts[2].x, possible[j].pts[0].x, possible[j].pts[2].x))
				if ((possible[i].rect & possible[j].rect).area() > 0)
					continue;
				if (!isBetween(min, max, min2, max2))
					continue;
				double currentDistBetween = fabs(possible[i].center.x - possible[j].center.x);
				if (currentDistBetween > bestDistBetween)
					continue;
				if (currentDistBetween > (camInfo->ImageHeight / 1.5))
					continue;

				if (verbose)
				{
					std::cout << "in between vert: " << isBetween(min, max, min2, max2) << std::endl;
				}

				bestDistBetween = currentDistBetween;
				possible[i].partner = j;
				possible[j].partner = i;
				cv::line(output, possible[i].center, possible[j].center, GREEN, 2);
				//}
			}
		}
	}

	if (possible.size() > 0)
	{
		double mostArea = 0, closest = 10000;
		double secondMostArea = 0, secondClosest = 10000;

		int firstBest = 0;
		int secondBest = 0;

		//Closest target and partner
		for (size_t i = 0; i < possible.size(); i++)
		{
			if (fabs(possible[i].center.x - camInfo->ImageHeight / 2) < closest)
			{
				firstBest = i;
				closest = fabs(possible[i].center.x - camInfo->ImageHeight / 2);
				secondBest = possible[i].partner;
			}
		}

		best.push_back(possible[firstBest]);
		if (firstBest != secondBest)
		{
			best.push_back(possible[secondBest]);
		}

		cv::RotatedRect rrect[2];
		bool isLeft[2];
		bool isRight[2];
		double area[2];
		double height[2];

		cv::Point2f rightCenter, leftCenter;
		if (verbose)
		{
			std::cout << "Total bests: " << best.size() << std::endl;
		}
		for (size_t i = 0; i < best.size(); i++)
		{
			rrect[i] = best[i].rRect;
			area[i] = best[i].rArea;
			if (best[i].rHeight >= best[i].rWidth)
			{
				height[i] = best[i].rHeight;
			}
			else
			{
				height[i] = best[i].rWidth;
			}

			cv::circle(output, best[i].center, 2, RED);
			//Point 0 is the lowest point, height is long edge from 0, width is short edge from 0
			for (unsigned int j = 0; j < 4; ++j)
			{
				if (!best[i].isLeft)
				{
					cv::line(output, best[i].pts[j], best[i].pts[(j + 1) % 4], PINK, 5);
					rightCenter = best[i].center;
					isLeft[i] = false;
					isRight[i] = true;
				}
				else
				{
					cv::line(output, best[i].pts[j], best[i].pts[(j + 1) % 4], YELLOW, 5);
					leftCenter = best[i].center;
					isLeft[i] = true;
					isRight[i] = false;
				}
			}
			//std::cout << "Position: " << center.x << std::endl;
		}

		if (best.size() == 2)
		{
			//Erase second best if not within 80% of best area
			/*if (area[1] < 0.8 * area[0])
				{
					best.erase(best.begin() + 1);
				}*/
			//Erase second best if too high/low compared to best y-pos
			if (rrect[1].center.y > (rrect[0].center.y) + (height[0] / 2) || rrect[1].center.y < (rrect[0].center.y) - (height[0] / 2))
			{
				best.erase(best.begin() + 1);
			}
			//Erase second best if same angle as best angle
			else if (isLeft[1] == isLeft[0] || isRight[1] == isRight[0])
			{
				best.erase(best.begin() + 1);
			}
			//Erase second best if on the wrong side of best based on angle
			else if ((rrect[1].center.x > rrect[0].center.x && isLeft[1]) || (rrect[1].center.x < rrect[0].center.x && !isLeft[1]))
			{
				best.erase(best.begin() + 1);
			}
			//Zero certain centers if not detected by this point
			if (best.size() == 1)
			{
				if (isLeft[0])
				{
					rightCenter.x = 0;
				}
				else
				{
					leftCenter.x = 0;
				}
			}
		}

		hasLeft = false, hasRight = false;
		if (leftCenter.x != 0 && rightCenter.x != 0)
		{
			targetX = (rightCenter.x + leftCenter.x) / 2;
			//targetY = (rightCenter.y + leftCenter.y) / 2;
			targetY = camInfo->ImageWidth - ((best[0].pts[2].y + best[1].pts[2].y) / 2);
			lastGoodLRSeparation = (rightCenter.x - leftCenter.x) / 2;
			leftTargetAngle = angleFromRawPixels(leftCenter.x) + camInfo->HorizMountAngle;
			rightTargetAngle = angleFromRawPixels(rightCenter.x) + camInfo->HorizMountAngle;
			bottomTargetAngle = angleFromRawPixels(targetY);
			targetsFound = 2;
			hasLeft = true, hasRight = true;
		}
		else
		{
			if (rightCenter.x != 0)
			{
				targetX = rightCenter.x - lastGoodLRSeparation;
				rightTargetAngle = angleFromRawPixels(rightCenter.x) + camInfo->HorizMountAngle;
				targetsFound = 1;
				hasRight = true;
			}
			if (leftCenter.x != 0)
			{
				targetX = leftCenter.x + lastGoodLRSeparation;
				leftTargetAngle = angleFromRawPixels(leftCenter.x) + camInfo->HorizMountAngle;
				targetsFound = 1;
				hasLeft = true;
			}
		}
	}

	if (hasLeft && hasRight)
	{
		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, camInfo->ImageWidth), GREEN, 2);
	}
	else
	{
		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, camInfo->ImageWidth), RED, 2);
	}
	cv::circle(output, cv::Point(camInfo->ImageHeight / 2, camInfo->ImageWidth / 2), 10, GREEN, 5);

	centeredTargetX = targetX - (camInfo->ImageWidth / 2);
	centeredTargetY = -targetY + (camInfo->ImageHeight / 2);
	targetAngle = angleFromPixels(centeredTargetX) + camInfo->HorizMountAngle;
}

bool TargetTracker2019::MergeTargets()
{
	bool changed = false;
	int size = contours.size();
	std::vector<int> rejects;
	for (size_t i = 0; i < size && size < 20; i++)
	{
		cv::RotatedRect rRect1 = cv::minAreaRect(contours[i]);
		cv::Point2f pts1[4];
		rRect1.points(pts1);
		double min = pts1[0].x;
		double max = pts1[2].x;
		for (size_t j = i + 1; j < size; j++)
		{
			cv::RotatedRect rRect2 = cv::minAreaRect(contours[j]);
			cv::Point2f pts2[4];
			rRect2.points(pts2);
			double min2 = pts2[0].x;
			double max2 = pts2[2].x;
			if (!isBetween(min, max, min2, max2))
				continue;

			//Convex hull around 2 contours
			//Push back convex hull
			//Erase position of two separate contours
			if (verbose)
			{
				std::cout << "in between horiz: " << isBetween(min, max, min2, max2) << std::endl;
			}
			std::vector<cv::Point> points, hull;

			points.insert(points.end(), contours[i].begin(), contours[i].end());
			points.insert(points.end(), contours[j].begin(), contours[j].end());
			cv::convexHull(cv::Mat(points), hull);

			contours.push_back(hull);

			if (std::find(rejects.begin(), rejects.end(), i) == rejects.end())
				rejects.push_back(i);
			if (std::find(rejects.begin(), rejects.end(), j) == rejects.end())
				rejects.push_back(j);
		}
	}

	//Must sort in order to ensure that we remove from the back
	std::sort(rejects.begin(), rejects.end());
	for (size_t i = rejects.size(); i > 0; i--)
	{
		contours.erase(contours.begin() + rejects[i - 1]);
		changed = true;
	}

	return changed;
}