#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/*
This is the version that was on my computer. It's around a month out of date compared with what actually ran on the robot since the last parts were written in the lab by the test arena.

Also, this was very old, using IplImage*. So I took the old code and ported it to OpenCV 4.5.5.

TODO - Though this worked reliably there are changes that could improve performance
	Find all 4 top corners
	Make this more robust by adding noise to image etc
	Smarter system for choosing the correct block
	Make thresholds robust to lighting changes
*/

// Colors for drawing
cv::Scalar red = CV_RGB(250, 0, 0);
cv::Scalar green = CV_RGB(0, 250, 0);
cv::Scalar blue = CV_RGB(0, 0, 250);
cv::Scalar white = CV_RGB(255, 255, 255);
cv::Scalar black = CV_RGB(0, 0, 0);
cv::Scalar yellow = CV_RGB(250, 250, 0);

// Setting thresholds
double hthresh = 220;          // 224
int satl = 75;                 // Saturation Low
int sath = 255;                // Saturation High
int huel = 0;
int hueh = 0;
int tolerance = 0;
int oraHalfRange = 50;
int yelHalfRange = 50;
int bluHalfRange = 4;
int greHalfRange = 3;
int broHalfRange = 5;
int redHalfRange = 5;
int oraAvg = 63;
int yelAvg = 51; // Clearly wrong, should be around 80-100
int bluAvg = 14;
int greAvg = 42;
int broAvg = 117;
int redAvg = 116;
int oras = 40;
int yels = 0;
int blus = 75;
int gres = 75;
int reds = 75;
int bros = 75;
int brohthresh = 50;
int orahl = oraAvg - oraHalfRange - tolerance; // 100 Before Android
int orahh = oraAvg + oraHalfRange + tolerance; // 116This area sets the upper and lower
int yelhl = yelAvg - yelHalfRange - tolerance; // 92
int yelhh = yelAvg + yelHalfRange + tolerance; // 102
int bluhl = bluAvg - bluHalfRange - tolerance; // 0
int bluhh = bluAvg + bluHalfRange + tolerance; // 17
int grehl = greAvg - greHalfRange - tolerance; // 47
int grehh = greAvg + greHalfRange + tolerance; // 90
int brohl = broAvg - broHalfRange - tolerance; // 110
int brohh = broAvg + broHalfRange + tolerance; // 130
int redhl = redAvg - redHalfRange - tolerance;
int redhh = redAvg + redHalfRange + tolerance;

int main() {
	
	// Color Threshold Values
	int in_color = 3; // What color block should be found
	if (in_color == 1) { // Brown
		hueh = brohh;
		huel = brohl;
		satl = bros;
		hthresh = brohthresh;
	}else if (in_color == 2) { // Red
		hueh = redhh; // 90
		huel = redhl; // 47
		satl = reds;
	}else if (in_color == 3) { // Orange
		hueh = orahh; // 116
		huel = orahl; // 108
		satl = oras;
	}else if (in_color == 4) { // Yellow
		hueh = yelhh; // 102
		huel = yelhl; // 92
		satl = yels;
	}else if (in_color == 5) { // Green
		hueh = grehh; // 90
		huel = grehl; // 47
		satl = gres;
	}else if (in_color == 6) { // Blue
		hueh = bluhh; // 17
		huel = bluhl; // 0
		satl = blus;
	}
	
	// Load Image and Initialize Values
	cv::Mat image = cv::imread("aligned.jpg");
	cv::Size size = image.size();
	if (size.width > 640) {
		float scale = 640.0 / size.width;
		int height_target = scale * size.height;
		cv::Size newsize = cv::Size(640, height_target);
		resize(image, image, newsize, 0, 0, 1);
		size = image.size();
	}
	cv::Point seed_point = cv::Point(size.width/2, size.height/2);
	cv::Size size_mask;
	size_mask.width = size.width + 2;
	size_mask.height = size.height + 2;
	cv::Mat mask(size_mask, CV_8UC1, cv::Scalar(0)); // Mask Image
	cv::Mat maskh(size_mask, CV_8UC1, cv::Scalar(0)); // H Mask Image
	cv::Mat masks(size_mask, CV_8UC1, cv::Scalar(0)); // S Mask Image
	cv::Mat imageg(size, CV_8UC1, cv::Scalar(0)); // Greyscale Image
	cv::Mat imagehsv(size, CV_8UC3, cv::Scalar(0, 0, 0)); // HSV Image
	cv::Mat imagecont(size, CV_8UC3, cv::Scalar(0, 0, 0)); // Contours Image
	std::vector<cv::Mat> channels;
	int erode_size = 7;
	cv::Mat erode_element = getStructuringElement(0,
		cv::Size(2 * erode_size + 1, 2 * erode_size + 1),
		cv::Point(erode_size, erode_size));
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours0;
	double maxContourArea = 0.0;
	double thisContourArea = 0.0;
	int maxAreaContourId = -1;
	cv::Rect br; // Bounding Rectangle
	double offset_x;
	cvtColor(image, imagehsv, cv::COLOR_RGB2HSV);
	split(imagehsv, channels);

	// Find Block Candidates
	threshold(channels[0], maskh, huel, hueh, 0);
	threshold(channels[1], masks, satl, sath, 0);
	bitwise_and(maskh, masks, mask);
	erode(mask, mask, erode_element);
	dilate(mask, mask, erode_element);
	findContours(mask, contours0, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	contours.resize(contours0.size());

	// Isolate THE Block
	double areaMin = 500;          // Defines an area below which to ignore (noise)
	double epsilon_orig = 15;      // Helps define most significant points from the approxPoly
	double epsilon = epsilon_orig;
	for (size_t k = 0; k < contours0.size(); k++) {
		epsilon = epsilon_orig;
		approxPolyDP(cv::Mat(contours0[k]), contours[k], epsilon, true); // https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c
		while (contours[k].size() < 4) {
			epsilon = epsilon / 2;
			cv::approxPolyDP(cv::Mat(contours0[k]), contours[k], epsilon / 2, true);
		}
		while (contours[k].size() > 4) {
			epsilon = epsilon + 5;
			cv::approxPolyDP(cv::Mat(contours0[k]), contours[k], epsilon / 2, true);
		}
		thisContourArea = contourArea(contours[k]);
		br = boundingRect(contours[k]);
		offset_x = pow((size.width / 2 - (br.x + br.width/2)), 2);
		if (thisContourArea > maxContourArea && thisContourArea > areaMin && offset_x < 2500) {
			maxContourArea = thisContourArea;
			maxAreaContourId = k;
		}
	}
	if (maxAreaContourId == -1) { // Fail Elegantly
		return 0;
	}
	
	// X Sort
	bool bDone = false; // This flag will be used to check whether we have to continue the algorithm
	double xTmp = 0; // New bubble sort method for arranging points by x values.
	double yTmp = 0;
	while (!bDone)
	{
		bDone = true; // Assume that the array is currently sorted

		for (int i = 0; i < contours[maxAreaContourId].size()-1; ++i) // For every corner found 
		{
			if (contours[maxAreaContourId][i].x > contours[maxAreaContourId][i+1].x) // Compare the current element with the following one
			{
				// They are in the wrong order, swap them
				xTmp = contours[maxAreaContourId][i].x;
				yTmp = contours[maxAreaContourId][i].y;
				contours[maxAreaContourId][i].x = contours[maxAreaContourId][i+1].x;
				contours[maxAreaContourId][i].y = contours[maxAreaContourId][i+1].y;
				contours[maxAreaContourId][i+1].x = xTmp;
				contours[maxAreaContourId][i+1].y = yTmp;

				bDone = false;
			}
		}
	}

	// Y Sort
	if (contours[maxAreaContourId][0].y < contours[maxAreaContourId][1].y) {
		xTmp = contours[maxAreaContourId][0].x;
		yTmp = contours[maxAreaContourId][0].y;
		contours[maxAreaContourId][0].x = contours[maxAreaContourId][1].x;
		contours[maxAreaContourId][0].y = contours[maxAreaContourId][1].y;
		contours[maxAreaContourId][1].x = xTmp;
		contours[maxAreaContourId][1].y = yTmp;
	}
	if (contours[maxAreaContourId][2].y < contours[maxAreaContourId][3].y) {
		xTmp = contours[maxAreaContourId][2].x;
		yTmp = contours[maxAreaContourId][2].y;
		contours[maxAreaContourId][2].x = contours[maxAreaContourId][3].x;
		contours[maxAreaContourId][2].y = contours[maxAreaContourId][3].y;
		contours[maxAreaContourId][3].x = xTmp;
		contours[maxAreaContourId][3].y = yTmp;
	}
	
	// Finding Block Orientation
	cv::Point lav[2];
	cv::Point hor[2];
	lav[0].x = (contours[maxAreaContourId][0].x + contours[maxAreaContourId][2].x) / 2;
	lav[0].y = (contours[maxAreaContourId][0].y + contours[maxAreaContourId][2].y) / 2;
	lav[1].x = (contours[maxAreaContourId][1].x + contours[maxAreaContourId][3].x) / 2;
	lav[1].y = (contours[maxAreaContourId][1].y + contours[maxAreaContourId][3].y) / 2;
	hor[0].x = (contours[maxAreaContourId][0].x + contours[maxAreaContourId][1].x) / 2;
	hor[0].y = (contours[maxAreaContourId][0].y + contours[maxAreaContourId][1].y) / 2;
	hor[1].x = (contours[maxAreaContourId][2].x + contours[maxAreaContourId][3].x) / 2;
	hor[1].y = (contours[maxAreaContourId][2].y + contours[maxAreaContourId][3].y) / 2;

	// Determine Block Size
	double rat = abs((double)lav[0].y - (double)lav[1].y) / abs((double)hor[0].x - (double)hor[1].x);

	// Draw
	for (int i = 0; i < contours[maxAreaContourId].size(); i++) {
		circle(image, contours[maxAreaContourId][i], 10, green, 3, 8, 0);
	}
	cv::line(image, lav[0], lav[1], red, 3, 8, 0);
	cv::line(image, hor[0], hor[1], red, 3, 8, 0);
	cv::imshow("image", image);
	int c = cv::waitKey(0);
	cv::destroyAllWindows();
}