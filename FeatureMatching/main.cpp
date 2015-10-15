#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用

using namespace cv;

void FeatureMatching(
	const std::string& filename1,			// 画像１のファイル名
	const std::string& filename2,			// 画像２のファイル名
	const std::string& featureDetectorName,		// detectorType
	const std::string& descriptorExtractorName,  // descriptorExtractorType
	const std::string& descriptorMatcherName,	// descriptorMatcherType
	bool crossCheck = false
	)				// マッチング結果をクロスチェックするかどうか
{
	// 画像の読み込み
	cv::Mat img1 = cv::imread(filename1);
	cv::Mat img2 = cv::imread(filename2);
	Mat src1, src2, dst1, dst2, color_dst1, color_dst2;

	GaussianBlur(img1, img1, Size(5, 5), 0, 0);
	GaussianBlur(img2, img2, Size(5, 5), 0, 0);
	src1 = img1.clone();
	src2 = img2.clone();
	Canny(src1, dst1, 50, 200, 3);
	cvtColor(dst1, color_dst1, CV_GRAY2BGR);
	Canny(src2, dst2, 50, 200, 3);
	cvtColor(dst2, color_dst2, CV_GRAY2BGR);
	color_dst1 = Mat(Size(img1.cols, img1.rows), CV_8U, Scalar::all(0));
	color_dst2 = Mat(Size(img2.cols, img1.rows), CV_8U, Scalar::all(0));
	
	vector<Vec4i> lines1;
	HoughLinesP(img1, lines1, 1, CV_PI / 180, 8, 5, 0);
	for (size_t i = 0; i < lines1.size(); i++)
	{
		line(color_dst1, Point(lines1[i][0], lines1[i][1]),
			Point(lines1[i][2], lines1[i][3]), Scalar(0, 0, 255));
	}
	/*
	vector<Vec4i> lines2;
	HoughLinesP(img2, lines2, 10, CV_PI / 180, 8, 5, 0);
	for (size_t i = 0; i < lines2.size(); i++)
	{
		line(color_dst2, Point(lines2[i][0], lines2[i][1]),
			Point(lines2[i][2], lines2[i][3]), Scalar(0, 0, 255));
	}*/
	
	// SIFT・SURFモジュールの初期化
	cv::initModule_nonfree();

	printf("A");

	// 特徴点抽出
	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(featureDetectorName);
	std::vector<cv::KeyPoint> keypoint1, keypoint2;
	detector->detect(img1, keypoint1);
	detector->detect(img2, keypoint2);
	//detector->detect(color_dst1, keypoint1);
	//detector->detect(color_dst2, keypoint2);

	printf("B");

	// 特徴記述
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(descriptorExtractorName);
	cv::Mat descriptor1, descriptor2;
	extractor->compute(img1, keypoint1, descriptor1);
	extractor->compute(img2, keypoint2, descriptor2);
	//extractor->compute(color_dst1, keypoint1, descriptor1);
	//extractor->compute(color_dst2, keypoint2, descriptor2);

	printf("C");

	// マッチング
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(descriptorMatcherName);
	std::vector<cv::DMatch> dmatch;
	/*if (crossCheck)
	{
	// クロスチェックする場合
	std::vector<cv::DMatch> match12, match21;
	matcher->match(descriptor1, descriptor2, match12);
	matcher->match(descriptor2, descriptor1, match21);
	for (size_t i = 0; i < match12.size(); i++)
	{
	cv::DMatch forward = match12[i];
	cv::DMatch backward = match21[forward.trainIdx];
	if (backward.trainIdx == forward.queryIdx)
	dmatch.push_back(forward);
	}
	}
	else
	{
	// クロスチェックしない場合
	matcher->match(descriptor1, descriptor2, dmatch);
	}*/

	// マッチング結果の表示
	cv::Mat out;
	cv::drawMatches(img1, keypoint1, img2, keypoint2, dmatch, out);
	//cv::drawMatches(color_dst1, keypoint1, color_dst2, keypoint2, dmatch, out);
	cv::imshow("matching", out);
	while (cv::waitKey(1) == -1);

	printf("\n終了");

}

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

