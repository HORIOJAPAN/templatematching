#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

# define _Evaluate 0
// 0:評価値基準の最適座標  1:自己位置座標を無視してマッチング率基準の最適座標

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/b102.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 2

# define _HyokaKizyun Evaluation1

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

float	kakudoHaba1 = 24;	// 1回目角度幅（片方向）
float	kakudoHaba2 = 8;	// 2回目

float	kizamiKakudo1 = 4;	// 1回目刻み角度
float	kizamiKakudo2 = 1;	// 2回目

Point maxPt;
Mat maxMatch;
float maxEvaluation = -100;
float maxEvaluation1 = -100;
float maxEvaluation2 = -100;
float maxEvaluation3 = -100;
int maxDistance;


/*
 * 変更する箇所
 * 
# define _ImageField "./img/fieldMap2.jpg"
 * ↑fieldMap.jpg と fieldMap2.jpg 後者が手書きで修正したもの．基本的に後者を使う
 * 
# define _ImageMatch "./img/a002.jpg"
 * ↑a,b,c 001~ , 101~ ただし，ディレクトリにないファイルを参照するとエラーになるのでよく見る
 * 
# define _MatchArea 1
 * ↑上で編集したマッチング画像の頭文字に応じて変更する．理想座標を変更する（自己位置の基準座標）
 * a:1 b:2 c:3
 * 
 */

/*
 * 評価値の計算方法
 * 引数は「tilt:相対角度」「dist:相対距離」「matchRatio:画像のマッチング率」
 * 角度は[deg]，距離は[pixel/5cm],マッチング率は 1 ~ 0
 * 角度と座標は実際のロボットの移動精度を考慮する
 * マッチング率は2枚の画像を載せたときにそのピクセルごとにアルかナイかを判定してる様子
 * それによって，元画像の線が太いと多少の角度は許容されるなど
 * Hyoka1ではマッチング率に桁をあわせて減点法で正負を判断しているが，これに縛られる必要はない
*/


void Hyoka1(float tilt, int dist, float matchRatio ,float& point){
	if (dist > 6){
		point = matchRatio - log10(dist) / 5;
	}
	else{
		point = matchRatio;
	}
}
void Hyoka2(float tilt, int dist, float matchRatio, float& point){
	point = matchRatio * 100 - dist;
}
void Hyoka3(float tilt, int dist, float matchRatio, float& point){
	point = 100 - dist - abs(tilt)*3 ;
}



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
	float distance;		// 理想座標とマッチング座標の相対距離
	float Evaluation1;	// 評価値
	float Evaluation2;	// 評価値
	float Evaluation3;	// 評価値
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
		// エンコーダによる自己位置とマッチング後の自己位置の相対距離を求める
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));


		// ↓評価値の計算はじめ
		Hyoka1(i, distance, maxVal, Evaluation1);
		Hyoka2(i, distance, maxVal, Evaluation2);
		Hyoka3(i, distance, maxVal, Evaluation3);
		// ↑評価値の計算おわり

		std::cout << (int)i << "\t" << maxVal << "   " << Pt.x << "," << Pt.y << "   " << (int)distance << "\t" <<
			Evaluation1 << "\t" << Evaluation2 << "\t" << Evaluation3 << std::endl;

		//printf("%d\t%lf\t%d,%d\n", (int)i, maxVal, Pt.x, Pt.y);

		if (maxVal > maxValue){
			ideal_x = Pt.x;
			ideal_y = Pt.y;
			maxValue = maxVal;
			Angle = angle;
		}
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
	float Evaluation1;	// 評価値
	float Evaluation2;	// 評価値
	float Evaluation3;	// 評価値
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


		Hyoka1(i, distance, maxVal, Evaluation1);
		Hyoka2(i, distance, maxVal, Evaluation2);
		Hyoka3(i, distance, maxVal, Evaluation3);


# if _Evaluate == 1
		if (maxVal > maxValue){
			maxValue = maxVal;
			Angle = angle;
			maxPt = Pt; 
			maxMatch = match;
		}
# else
		if (_HyokaKizyun > maxEvaluation){
			maxValue = maxVal;
			Angle = angle;
			maxPt = Pt; 
			maxMatch = match;
			maxDistance = distance;
			maxEvaluation = Evaluation1;
			maxEvaluation1 = Evaluation1;
			maxEvaluation2 = Evaluation2;
			maxEvaluation3 = Evaluation3;
		}
# endif
		std::cout << (int)i << "\t" << maxVal<< "   " << Pt.x<< "," << Pt.y << "   " << (int)distance << "\t" <<
			Evaluation1 << "\t" << Evaluation2 << "\t" << Evaluation3 << std::endl;
	}
}


int main()
{
	Mat img1 = imread(_ImageField);
	Mat img2 = imread(_ImageMatch);
	//Mat img1 = imread("./img/fieldMap2.jpg");
	//Mat img2 = imread("./img/c109.jpg");
	Mat kaitenImg;

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

	printf("%d度ずつ刻んでマッチ開始\n", (int)kizamiKakudo1);
	printf("相対度\t相関値\t   x,y\t   距離\t評価値\n");
	MatchingEvaluation(sub, img2, 0, kakudoHaba1, kizamiKakudo1);

	/*
	if (D == 0.0){
		printf("%f\n", D);
		printf("No matching\n");
		return(0);
	}
	*/

	printf("%d度ずつ刻んでマッチ開始\n", (int)kizamiKakudo2);
	printf("相対度\t相関値\t  x,y\t   距離\t評価値\n");

	D = 0.0;
	MatchingEvaluation(sub, img2, Angle, kakudoHaba2, kizamiKakudo2);

	float angle = Angle;
	float scale = 1.0;
	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	//matchTemplate(sub, kaitenImg, match, CV_TM_CCOEFF_NORMED);
	//minMaxLoc(maxMatch, NULL, &maxValue, NULL, &maxPt);
	rectangle(sub , maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	imshow("kaitenImg", kaitenImg);

	std::cout << "\n" << (int)angle << "\t" << maxValue << "    " << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "   " <<
		maxEvaluation1 << "\t" << maxEvaluation2 << "\t" << maxEvaluation3 << std::endl;
	printf("%ld[ms]\n" , clock()-start);

	imshow("Image", sub);
	waitKey(0);
}

