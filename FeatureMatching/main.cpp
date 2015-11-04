#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用
#include <time.h>
#include <math.h>
#include <iostream>

# define PI 3.1415926

using namespace cv;

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/a108.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 1

// グローバル環境イメージのサイズ指定
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

// 回転行列に用いる倍率（1倍に固定）
# define scale 1

float	kakudoHaba0 = 45;	// テスト処理用
float	kakudoHaba1 = 24;	// 1回目角度幅（片方向）
float	kakudoHaba2 = 5;	// 2回目

float	kizamiKakudo0 = 5;	// テスト処理用
float	kizamiKakudo1 = 3;	// 1回目刻み角度
float	kizamiKakudo2 = 1;	// 2回目

// 最大の点での各値をmax付きの変数に格納
Point maxPt;
Mat maxMatch;
// Eva0はマッチング率のみの生データ
double	maxEvaluation0 = 0;
float	maxAngle = 0.0;
// 採用する評価値を代入
float maxEvaluation = -10000;
// 他の評価方法での出力用の参考値
float maxEvaluation1 = -10000;
// 最大の点での理想点との相対距離
int maxDistance;


void Hyoka1(float tilt, int dist, float matchRatio, float& score){
	score = (matchRatio * 100 - dist / 5 * (cos(tilt * PI / 360) + 1));
}


void MatchingEvaluation(const cv::Mat img1,			// グローバル環境イメージ（大）
	const cv::Mat img2,			// ローカル環境イメージ（小）
	float angle_center,			// 幅の中心
	float angleHalfRange,		// 角度幅
	float angleDelta			// 刻み角度
	)
{
	// テンプレートマッチングと評価値の算出・最適値の出力

	Mat match;		// マッチング率の配列
	Mat kaitenImg;	// 回転させた地図を格納
	Point Pt;		// 画像内での最高マッチング率の座標を格納
	float distance;		// 理想座標とマッチング座標の相対距離
	float Evaluation1;	// 評価値

	maxEvaluation0 = 0;
	maxAngle = 0.0;
	maxEvaluation = -10000;
	maxEvaluation1 = -10000;
	maxDistance = 0;

	std::cout << "　■　中央角度：" << (int)angle_center << "[deg]　ピッチ：" << (int)angleDelta << "[deg]　　マッチング開始\n" << "回転角\t相関値   \tx,y\t距離\t評価値" << std::endl;

	for (float i = (angle_center)-(angleHalfRange); i < ((angle_center)+(angleHalfRange)+1); i += angleDelta)
	{
		// 回転角[deg]
		float angle = i;

		// 画像の中心を求める
		// 回転行列の定義
		// 画像を回転
		// 回転後画像をウィンドウ表示
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		warpAffine(img2, kaitenImg, matrix, img2.size());

		// テンプレートマッチング
		// 相関値の配列から最大値を抽出する
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);

		for (int k = 0; k < match.cols; k++)
		{
			for (int j = 0; j < match.rows; j++)
			{
				if (match.at<float>(j, k) > 0.1)
				{// マッチング率0.1を超えたときのみ評価処理
					distance = sqrt((k - ideal_x)*(k - ideal_x) + (j - ideal_y)*(j - ideal_y));
					Hyoka1(angle, distance, match.at<float>(j, k), Evaluation1);

					if (Evaluation1 > maxEvaluation){
						maxEvaluation0 = match.at<float>(j, k);
						maxAngle = angle;
						maxPt.x = k;
						maxPt.y = j;
						maxDistance = distance;
							
						maxEvaluation = Evaluation1;
					}
				}
			}
		}
		std::cout << (int)i << "\t" << maxEvaluation0 << "   \t" << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "\t" << maxEvaluation << std::endl;
	}
	std::cout << std::endl;
}


int main()
{
	clock_t start = clock();
	std::cout << "評価基準：1" << "\n" << std::endl;

	// 画像の配列を宣言
	Mat img1 = imread(_ImageField);
	Mat img2 = imread(_ImageMatch);
	//Mat img1 = imread("./img/fieldMap2.jpg");
	//Mat img2 = imread("./img/c109.jpg");
	Mat kaitenImg;

	// 理想座標の指定（エンコーダやサーボの指定値による）
	ideal_x -= leftMargin;
	ideal_y -= upMargin;

	// 画像img1からマッチング対象領域を指定して再定義
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (1000 - leftMargin - rightMargin), (1000 - upMargin - downMargin)));


	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba1, kizamiKakudo1);

	// この辺でマッチング失敗を返す？

	MatchingEvaluation(fieldMap, img2, maxAngle, kakudoHaba2, kizamiKakudo2);

	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, maxAngle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	rectangle(fieldMap, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);

	std::cout << "相対度\t相関値   \tx,y\t距離\t評価値" << std::endl;
	std::cout << (int)maxAngle << "\t" << maxEvaluation0 << "   \t" << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "\t" <<
		maxEvaluation << std::endl;
	std::cout << "処理時間：" << clock() - start << "[ms]" << std::endl;

	imshow("Image", fieldMap);
	imshow("kaitenImg", kaitenImg);
	waitKey(0);
}

