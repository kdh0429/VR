#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int ac, char** av) {

	cv::VideoCapture cap(2);
	
	if (!cap.isOpened())
	{
		printf("Can't open the camera");
		return -1;
	}

	Mat img;

	while (1)
	{
		cap >> img;

		imshow("camera img", img);

		if (waitKey(1) == 27)
			break;
	}


	return 0;
}