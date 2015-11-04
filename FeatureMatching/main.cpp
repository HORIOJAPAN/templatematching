#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>

using namespace cv;

int main()
{

	Mat img1 = imread("rouka02.jpg");
	Mat img2 = imread("rouka002.jpg");
	Mat match;	
	Mat kaitenImg;
	Point Pt;
	Point maxPt;
	float Angle;
	double minVal , maxVal;
	double minValue = 0 , maxValue = 0;

	imshow("Img", img2);

	printf("開始\n");

	for (int i = -5; i < 5; i += 1){

		// 回転：  [deg]
		float angle = i ;
		// 大きさ：  [倍]
		float scale = 1.0;
		// 画像の中心を求める
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		// 回転行列
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		// 画像を回転
		warpAffine(img2, kaitenImg, matrix, img2.size());
		imshow("kaitenImg", kaitenImg);
		// マッチング
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// 相関値を求める
		minMaxLoc(match, &minVal, &maxVal, NULL, &Pt);
		//printf("%lf %lf\n", minVal, maxVal);
		if (maxVal > maxValue){
			maxValue = maxVal;
			minValue = minVal;
			Angle = angle;
		}
		//printf( "%lf %lf\n", minValue, maxValue );
	}

	float angle = Angle;
	float scale = 1.0;
	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	imshow("kaitenImg", kaitenImg);
	matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
	minMaxLoc(match, &minValue, &maxValue, NULL, &maxPt);
	rectangle(img1, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	//rectangle(img1, maxPt, Point( maxPt.x , maxPt.y ), Scalar(0, 0, 255), 2, 8, 0);
	printf("Angle = %f position.x = %d position.y = %d \n", Angle, maxPt.x , maxPt.y);
	imshow("Image", img1);
	waitKey(0);

}

