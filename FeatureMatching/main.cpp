#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT・SURFモジュール用
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

# define PI 3.1415926

using namespace cv;

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/c103.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 3

// # define _HyokaNumber 0 : 評価基準1,2,3のどれにするか，0にするとマッチング率のみで処理
# define _HyokaNumber 3

# if _HyokaNumber == 1
# define _HyokaKizyun Evaluation1
# elif _HyokaNumber == 2
# define _HyokaKizyun Evaluation2
# elif _HyokaNumber == 3
# define _HyokaKizyun Evaluation3
# else
# define _HyokaKizyun Evaluation0
# endif

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
float	kakudoHaba1 = 18;	// 1回目角度幅（片方向）
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
float maxEvaluation2 = -10000;
float maxEvaluation3 = -10000;
// 最大の点での理想点との相対距離
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
 * 引数は「tilt:相対角度」「dist:相対距離」「matchRatio:画像のマッチング率」「score:評価値を返すポインタ」
 * 角度は[deg]，距離は[pixel/5cm],マッチング率は 1 ~ 0
 * 角度と座標は実際のロボットの移動精度を考慮する
 * マッチング率は2枚の画像を載せたときにそのピクセルごとにアルかナイかを判定してる様子
 * それによって，元画像の線が太いと多少の角度は許容されるなど
 * Hyoka1ではマッチング率に桁をあわせて減点法で正負を判断しているが，これに縛られる必要はない
*/


void Hyoka1(float tilt, int dist, float matchRatio ,float& score){
	if (dist > 6){
		score = matchRatio - log10(dist) / 5;
	}
	else{
		score = matchRatio;
	}
}
void Hyoka2(float tilt, int dist, float matchRatio, float& score){
	score = 100 - dist / matchRatio;
}
void Hyoka3(float tilt, int dist, float matchRatio, float& score){
	score = 100 - pow(dist,0.7) / matchRatio * (cos(tilt * PI / 360) + 1) / 2;
}


void MatchingEvaluation(	const cv::Mat img1,			// グローバル環境イメージ（大）
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
	double Evaluation0;	// 生マッチング率の最大値
	float Evaluation1;	// 評価値
	float Evaluation2;	// 評価値
	float Evaluation3;	// 評価値

	// printf("%d度ずつ刻んでマッチ開始\n", (int)angleDelta);
	// printf("相対度\t相関値   \tx,y\t距離\t評価値\n");
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
		imshow("kaitenImg", kaitenImg);

		// テンプレートマッチング
		// 相関値の配列から最大値を抽出する
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		minMaxLoc(match, NULL, &Evaluation0, NULL, &Pt);

		// 理想の自己位置座標と最適マッチング座標との相対距離の算出
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));

		// 各種評価値の計算
		Hyoka1(angle, distance, Evaluation0, Evaluation1);
		Hyoka2(angle, distance, Evaluation0, Evaluation2);
		Hyoka3(angle, distance, Evaluation0, Evaluation3);
		// （match配列の全要素に対して評価処理すべき）

		if (_HyokaKizyun > maxEvaluation){
			maxEvaluation0 = Evaluation0;
			maxAngle = angle;
			maxPt = Pt;
			maxDistance = distance;

			maxEvaluation = _HyokaKizyun;
			maxEvaluation1 = Evaluation1;
			maxEvaluation2 = Evaluation2;
			maxEvaluation3 = Evaluation3;
		}

		std::cout << (int)i << "\t" << Evaluation0<< "   \t" << Pt.x<< "," << Pt.y << "\t" << (int)distance << "\t" <<
			Evaluation1 << "   \t" << Evaluation2 << "   \t" << Evaluation3 << std::endl;
	}
	std::cout << std::endl;
}


int main()
{
	clock_t start = clock();
	std::cout << "評価基準：" << _HyokaNumber << "\n" << std::endl;

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
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (1000 - leftMargin - rightMargin), (1000 - upMargin - downMargin)) );


	if (!_HyokaNumber){
		printf("マッチング率から自己位置を見つけています...\n", kizamiKakudo0);
		MatchingEvaluation(fieldMap, img2, 0, kakudoHaba0, kizamiKakudo0);
	}

	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba1, kizamiKakudo1);

	/* マッチング失敗を返す
	if (D == 0.0){
		printf("%f\n", D);
		printf("No matching\n");
		return(0);
	}
	*/

	MatchingEvaluation(fieldMap, img2, maxAngle, kakudoHaba2, kizamiKakudo2);

	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, maxAngle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	rectangle(fieldMap, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	imshow("rawImg", img2);
	imshow("kaitenImg", kaitenImg);


	std::cout << "相対度\t相関値   \tx,y\t距離\t評価値" << std::endl;
	std::cout << (int)maxAngle << "\t" << maxEvaluation0 << "   \t" << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "\t" <<
		maxEvaluation1 << "   \t" << maxEvaluation2 << "   \t" << maxEvaluation3 << std::endl;
	// printf("%ld[ms]\n" , clock()-start);
	std::cout << "処理時間：" << clock() - start << "[ms]" << std::endl;

	imshow("Image", fieldMap);
	waitKey(0);
}

