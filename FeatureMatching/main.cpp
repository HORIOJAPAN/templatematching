#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用
#include <time.h>
#include <math.h>
#include <iostream>

using namespace cv;

int sp_x;
int sp_y;
int sp_angle;

float	kakudoHaba1 = 24;	// 1回目角度幅（片方向）
float	kakudoHaba2 = 5;	// 2回目

float	kizamiKakudo1 = 3;	// 1回目刻み角度
float	kizamiKakudo2 = 1;	// 2回目

void Hyoka1(float tilt, float dist, float matchRatio, float& score){
	score = (matchRatio * 100 - dist / 5 * (cos(tilt * 3.1415926 / 360) + 1));
}

void MatchingEvaluation(
	const cv::Mat img1,			// グローバル環境イメージ（大）
	const cv::Mat img2,			// ローカル環境イメージ（小）
	float &angle_center,			// 幅の中心
	float angleHalfRange,		// 角度幅
	float angleDelta,			// 刻み角度
	int &ideal_x,
	int &ideal_y
	)
{
	// テンプレートマッチングと評価値の算出・最適値の出力

	Mat match;		// マッチング率の配列
	Mat kaitenImg;	// 回転させた地図を格納
	Point Pt;		// 画像内での最高マッチング率の座標を格納
	float distance;		// 理想座標とマッチング座標の相対距離
	float Evaluation1;	// 評価値

	double maxEvaluation0 = 0;
	float maxAngle = angle_center;
	float maxEvaluation = -10000;
	float maxDistance = 0;

	std::cout << "　■　中央角度：" << (int)angle_center << "[deg]　ピッチ：" << (int)angleDelta << "[deg]　　マッチング開始\n" << "回転角\t相関値   \tx,y\t距離\t評価値" << std::endl;

	for (float angle = (angle_center - angleHalfRange); angle < (angle_center + angleHalfRange + 1); angle += angleDelta)
	{	// 中央角‐範囲半角　から　中央角＋範囲半角　まで　1[deg]刻み

		// 画像の中心を求める
		// 回転行列の定義
		// 画像を回転
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		Mat matrix = cv::getRotationMatrix2D(center, angle, 1);
		warpAffine(img2, kaitenImg, matrix, img2.size());

		// テンプレートマッチング
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);

		for (int k = 0; k < match.cols; k++)
		{
			for (int j = 0; j < match.rows; j++)
			{
				distance = sqrt((k - ideal_x)*(k - ideal_x) + (j - ideal_y)*(j - ideal_y));
				Hyoka1(angle, distance, match.at<float>(j, k), Evaluation1);

				if (Evaluation1 > maxEvaluation){
					maxEvaluation0 = match.at<float>(j, k);
					maxAngle = angle;
					Pt = Point(k, j);
					maxDistance = distance;
					maxEvaluation = Evaluation1;
				}
			}
		}
		std::cout << (int)angle << "\t" << maxEvaluation0 << "   \t" << Pt.x << "," << Pt.y << "\t" << (int)maxDistance << "\t" << maxEvaluation << std::endl;
	}
	std::cout << std::endl;

	ideal_x = Pt.x;
	ideal_y = Pt.y;
	angle_center = maxAngle;
}

// 外側をマッチング対象としないとき、外側からの率を指定
# define _RightMargin 0
# define _LeftMargin 0
# define _UpMargin 0.3
# define _DownMargin 0.3


void spEstimate(int &ideal_x, int &ideal_y , int &ideal_angle , string imageName)
{ // Self Position Estimate
	std::cout << "評価基準：1" << "\n" << std::endl;

	// 画像の配列を宣言
	Mat img1 = imread(imageName);
	Mat img2 = imread("./img/c109.jpg");
	Mat kaitenImg;

	const int rightMargin = img1.cols * _RightMargin;
	const int leftMargin = img1.cols * _LeftMargin;
	const int upMargin = img1.rows * _UpMargin;
	const int downMargin = img1.rows * _DownMargin;


	Point ideal_Pt;
	float tempAngle = ideal_angle;

	// 理想座標の指定（エンコーダやサーボの指定値による）
	ideal_x -= leftMargin;
	ideal_y -= upMargin;

	// 画像img1からマッチング対象領域を指定して再定義
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (img1.cols - leftMargin - rightMargin), (img1.rows - upMargin - downMargin)));

	ideal_Pt = Point(ideal_x, ideal_y);
	tempAngle = ideal_angle;
	MatchingEvaluation(fieldMap, img2, tempAngle, kakudoHaba1, kizamiKakudo1, ideal_Pt.x, ideal_Pt.y);
	// この辺でマッチング失敗を返す？

	ideal_Pt = Point(ideal_x, ideal_y);
	tempAngle = ideal_angle;
	MatchingEvaluation(fieldMap, img2, tempAngle, kakudoHaba2, kizamiKakudo2, ideal_Pt.x, ideal_Pt.y);

	ideal_x = ideal_Pt.x;
	ideal_y = ideal_Pt.y;
	ideal_angle = tempAngle;

	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, tempAngle, 1);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	rectangle(fieldMap, ideal_Pt, Point(ideal_Pt.x + kaitenImg.cols, ideal_Pt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);

	imshow("Image", fieldMap);
	imshow("kaitenImg", kaitenImg);
	imshow("rawImg", img2);
}

// _MatchArea => a:1 b:2 c:3
# define _MatchArea 3

int ideal_angle = 0;

# if _MatchArea == 1
int		ideal_x = 175;
int		ideal_y = 403;
# elif _MatchArea == 2
int		ideal_x = 374;
int		ideal_y = 423;
# elif _MatchArea == 3
int		ideal_x = 611;
int		ideal_y = 458;
# else
int		ideal_x = _FieldHeight / 2;
int		ideal_y = _FieldWidth / 2;
# endif


void main(){
	clock_t start = clock();

	sp_x = ideal_x;
	sp_y = ideal_y;
	sp_angle = ideal_angle;
	string imageName = "./img/fieldMap2.jpg";

	spEstimate(sp_x, sp_y, sp_angle, imageName);

	std::cout << "相対度\tx,y\n" << sp_angle << "   \t" << sp_x << "," << sp_y << std::endl;
	std::cout << "処理時間：" << clock() - start << "[ms]" << std::endl;

	waitKey(0);
}
