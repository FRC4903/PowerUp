#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/*
 * Initial variables
 * for trackbars
 */

// Hue
int H_MIN = 0;
int H_MAX = 256;

// Saturation
int S_MIN = 0;
int S_MAX = 256;

// Value
int V_MIN = 0;
int V_MAX = 256;

// window diminsion variables
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

// max number of objects to be detected in frame
const int MAX_OBJECTS = 50;

// minimum and maximum object area in pixles
const int MIN_OBJECT_AREA = 400;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

// Titles that will appear at the top of each window
const std::string windowName = "Original Image";
const std::string windowName1 = "HSV Image";
const std::string windowName2 = "Thresholded Image";
const std::string windowName3 = "After Morphological Operations";
const std::string trackbarWindowName = "Trackbars";

/*
 * This function gets called whenever a
 * trackbar position is changed
 */
void on_trackbar(int, void*){}

/*
 * Trackbar window
 */
void createTrackbars()
{
    // creates a window, the first param is
    // the window size the second is a special
    // flag that we don't need to worry about for now
    cv::namedWindow(trackbarWindowName, 0);

    // ceates all the track bars in the trackbar window with theire respective
    // HSV values
    cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

/*
 * This function draws the tracking
 * pointer on the object in the original
 * image window
 */
void drawObject(int x, int y, cv::Mat &frame)
{
    // draws a circle on the given frame in the parameter with the
    // given dimensions and location
	circle(frame, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);

    // The following statements act as sentinels for the pointer to
    // track the object even if its partly out of the captured image
    // and the window frame
    if(y - 25 > 0)
        line(frame, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(0, 255, 0), 2);
    else line(frame, cv::Point(x, y), cv::Point(x, 0), cv::Scalar(0, 255, 0), 2);

    if(y + 25 < FRAME_HEIGHT)
        line(frame, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(0, 255, 0), 2);
    else line(frame, cv::Point(x, y), cv::Point(x, FRAME_HEIGHT), cv::Scalar(0, 255, 0), 2);

    if(x - 25 > 0)
        line(frame, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(0, 255, 0), 2);
    else line(frame, cv::Point(x, y), cv::Point(0, y), cv::Scalar(0, 255, 0), 2);

    if(x + 25 < FRAME_WIDTH)
        line(frame, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(0, 255, 0), 2);
    else line(frame, cv::Point(x, y), cv::Point(FRAME_WIDTH, y), cv::Scalar(0, 255, 0), 2);

}

/*
 * dilation and erosion
 * morphological operations
 */
void morphOps(cv::Mat &thresh)
{
	// uses given shape to dilate the registered threshholds
	cv::Mat erodeElement = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    // dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
}

void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed)
{
	cv::Mat temp;
	threshold.copyTo(temp);
	// these two vectors needed for output of findContours
    std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	// find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
	// use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
    {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_OBJECTS we have a noisy filter
        if(numObjects < MAX_OBJECTS)
        {
			for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
				cv::Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea)
                {
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}
                else objectFound = false;
			}
			//let user know you found an object
			if(objectFound == true)
            {
				putText(cameraFeed,"Tracking Object", cv::Point(0,50), 2, 1, cv::Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x, y, cameraFeed);
            }

		}
        else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER", cv::Point(0,50), 1, 2, cv::Scalar(0,0,255), 2);
	}
}

int main(int argc, char* argv[])
{
	// some boolean variables for different functionality within this
	// program
    bool trackObjects = true;
    bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
    cv::Mat cameraFeed;
	//matrix storage for HSV image
    cv::Mat HSV;
	//matrix storage for binary threshold image
	cv::Mat threshold;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//create slider bars for HSV filtering
	createTrackbars();
	//video capture object to acquire webcam feed
	cv::VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(1);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1)
    {
		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, cv::COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		cv::inRange(HSV,  cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
		morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects)trackFilteredObject(x, y, threshold, cameraFeed);

		//show frames
		imshow(windowName2, threshold);
		imshow(windowName, cameraFeed);
		imshow(windowName1, HSV);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		cv::waitKey(10);
	}

	return 0;
}
