#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

# define _Evaluate 0
// 0:自己位置を直接入力  1:相関値のみで自己位置を推定
// 0:テンプレートマッチングのみ  1:評価値計算

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/a002.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 1

# define _FieldHeight 1000
# define _FieldWidth 1000

// 外側をマッチング対象としないとき、外側からの率を指定
# define _RightMargin 0
# define _LeftMargin 0
# define _UpMargin 0.3
# define _DownMargin 0.3

const int rightMargin = _FieldWidth * _RightMargin;
const int leftMargin = _FieldWidth * _LeftMargin;
const int upMargin = _FieldHeight * _UpMargin;
const int downMargin = _FieldHeight * _DownMargin;

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

double	maxValue = 0;
float	Angle = 0.0;
float	D = 0.0;



void Localization(		const cv::Mat img1,			// 画像１のファイル名
						const cv::Mat img2,			// 画像２のファイル名
						float angle_center,			// 幅の中心
						float angle_width,			// 角度幅
						float angle_shredded		// 刻み角度				
						)
{
	// テンプレートマッチングと評価値の算出・最適値の出力

	Mat match;
	Mat kaitenImg;
	Point Pt;
	double maxVal;		// マッチング率の最大値

	for (float i = (angle_center)-(angle_width); i < ((angle_center) + (angle_width) + 1) ; i += angle_shredded){
		// 回転：  [deg]
		float angle = i;
		// 大きさ：  [倍]
		float scale = 1.0;
		// 画像の中心を求める
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		// 回転行列
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		// 画像を回転
		warpAffine(img2, kaitenImg, matrix, img2.size());
		// ウィンドウ表示
		imshow("kaitenImg", kaitenImg);
		// マッチング
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// 相関値を求める
		minMaxLoc(match, NULL, &maxVal, NULL, &Pt);

		//printf("%d\t%lf\t%d,%d\n", (int)i, maxVal, Pt.x, Pt.y);

		if (maxVal > maxValue){
			ideal_x = Pt.x;
			ideal_y = Pt.y;
			maxValue = maxVal;
			Angle = angle;
		}
		std::cout << (int)i << "\t" << maxVal << "\t" << Pt.x << "," << Pt.y << std::endl;
	}
	printf("%f\t%lf\t%d,%d\n", Angle, maxValue, ideal_x, ideal_y);
}

void MatchingEvaluation(	const cv::Mat img1,			// 画像１のファイル名
							const cv::Mat img2,			// 画像２のファイル名
							float angle_center,			// 幅の中心
							float angle_width,			// 角度幅
							float angle_shredded		// 刻み角度				
							)
{
	// テンプレートマッチングと評価値の算出・最適値の出力

	Mat match;
	Mat kaitenImg;
	Point Pt;
	float distance;		// 理想座標とマッチング座標の相対距離
	float Evaluation;	// 評価値
	double maxVal;		// マッチング率の最大値
	// double maxVal_1 = 0, maxVal_2 = 0;

	for (float i = (angle_center)-(angle_width); i < ((angle_center)+(angle_width)+1); i += angle_shredded){
		// 回転：  [deg]
		float angle = i;
		// 大きさ：  [倍]
		float scale = 1.0;
		// 画像の中心を求める
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		// 回転行列
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		// 画像を回転
		warpAffine(img2, kaitenImg, matrix, img2.size());
		// ウィンドウ表示
		imshow("kaitenImg", kaitenImg);
		// テンプレートマッチング
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// 相関値を求める
		minMaxLoc(match, NULL, &maxVal, NULL, &Pt);
		// エンコーダによる自己位置とマッチング後の自己位置の相対距離を求める
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));


		// ↓評価値の計算はじめ
		if (distance > 6){ // 距離が6（6ピクセル：30cm）未満のとき
			Evaluation = maxVal - log10(distance) / 5;
		}
		else{
			Evaluation = maxVal;
		}
		// ↑評価値の計算おわり

		// printf("%d\t%lf\t%d,%d \t%d\t%lf\n", (int)i, maxVal, Pt.x, Pt.y, (long)distance, Evaluation);

		std::cout << (int)i << "\t" << maxVal<< "\t" << Pt.x<< "," << Pt.y <<
			"\t" << (int)distance<< "\t" << Evaluation << std::endl;

# if _Evaluate == 0
		if (maxVal > maxValue){
			maxValue = maxVal;
			Angle = angle;
		}
# else
		if (Evaluation > 0){
			if (D < Evaluation){
				maxValue = maxVal;
				Angle = angle;
				D = Evaluation;
			}
		}
# endif
	}
}


int main()
{
	Mat img1 = imread(_ImageField);
	Mat img2 = imread(_ImageMatch);
	//Mat img1 = imread("./img/fieldMap2.jpg");
	//Mat img2 = imread("./img/c109.jpg");
	Mat match;	
	Mat kaitenImg;
	Point Pt;
	Point maxPt;
	float ex_angle = 0.0;
	double maxValue = 0;

	ideal_x -= leftMargin;
	ideal_y -= upMargin;

	clock_t start = clock();

	// img1の画像の領域を指定
	Mat sub = img1(Rect(leftMargin, upMargin, 1000 - leftMargin - rightMargin, 1000 - upMargin - downMargin));
	//imshow("Img", img2);

# if _Evaluate

	int m = 5;
	printf("自己位置を見つけています...\n", m);
	Localization(sub, img2, 0, 45, m);

# endif

	int n = 5;
	printf("%d度ずつ刻んでマッチ開始\n" , n);
	printf("度\t相関値\t\tx,y\t距離\t評価値\n");
	MatchingEvaluation(sub, img2, 0, 45, n);

# if _Evaluate
	if (D == 0.0){
		printf("%f\n", D);
		printf("No matching\n");
		return(0);
	}
# endif

	printf("1度ずつ刻んでマッチ開始\n");
	printf("度\t相関値\t\tx,y\t\t距離\t評価値\n");

	D = 0.0;
	MatchingEvaluation(sub, img2, Angle, 5, 1);

	float angle = Angle;
	float scale = 1.0;
	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	matchTemplate(sub, kaitenImg, match, CV_TM_CCOEFF_NORMED);
	minMaxLoc(match, NULL, &maxValue, NULL, &maxPt);
	rectangle(sub , maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	imshow("kaitenImg", kaitenImg);
	printf("Angle = %f position.x = %d position.y = %d \n", angle, maxPt.x , maxPt.y);
	printf("%ld\n" , clock()-start);
	imshow("Image", sub);
	waitKey(0);
}

