#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>
#include <math.h>
#include <iostream>

using namespace cv;

int sp_x;
int sp_y;
int sp_angle;

float	kakudoHaba1 = 24;	// 1��ڊp�x���i�Е����j
float	kakudoHaba2 = 5;	// 2���

float	kizamiKakudo1 = 3;	// 1��ڍ��݊p�x
float	kizamiKakudo2 = 1;	// 2���

void Hyoka1(float tilt, float dist, float matchRatio, float& score){
	score = (matchRatio * 100 - dist / 5 * (cos(tilt * 3.1415926 / 360) + 1));
}

void MatchingEvaluation(
	const cv::Mat img1,			// �O���[�o�����C���[�W�i��j
	const cv::Mat img2,			// ���[�J�����C���[�W�i���j
	float &angle_center,			// ���̒��S
	float angleHalfRange,		// �p�x��
	float angleDelta,			// ���݊p�x
	int &ideal_x,
	int &ideal_y
	)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;		// �}�b�`���O���̔z��
	Mat kaitenImg;	// ��]�������n�}���i�[
	Point Pt;		// �摜���ł̍ō��}�b�`���O���̍��W���i�[
	float distance;		// ���z���W�ƃ}�b�`���O���W�̑��΋���
	float Evaluation1;	// �]���l

	double maxEvaluation0 = 0;
	float maxAngle = angle_center;
	float maxEvaluation = -10000;
	float maxDistance = 0;

	std::cout << "�@���@�����p�x�F" << (int)angle_center << "[deg]�@�s�b�`�F" << (int)angleDelta << "[deg]�@�@�}�b�`���O�J�n\n" << "��]�p\t���֒l   \tx,y\t����\t�]���l" << std::endl;

	for (float angle = (angle_center - angleHalfRange); angle < (angle_center + angleHalfRange + 1); angle += angleDelta)
	{	// �����p�]�͈͔��p�@����@�����p�{�͈͔��p�@�܂Ł@1[deg]����

		// �摜�̒��S�����߂�
		// ��]�s��̒�`
		// �摜����]
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		Mat matrix = cv::getRotationMatrix2D(center, angle, 1);
		warpAffine(img2, kaitenImg, matrix, img2.size());

		// �e���v���[�g�}�b�`���O
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

// �O�����}�b�`���O�ΏۂƂ��Ȃ��Ƃ��A�O������̗����w��
# define _RightMargin 0
# define _LeftMargin 0
# define _UpMargin 0.3
# define _DownMargin 0.3


void spEstimate(int &ideal_x, int &ideal_y , int &ideal_angle , string imageName)
{ // Self Position Estimate
	std::cout << "�]����F1" << "\n" << std::endl;

	// �摜�̔z���錾
	Mat img1 = imread(imageName);
	Mat img2 = imread("./img/c109.jpg");
	Mat kaitenImg;

	const int rightMargin = img1.cols * _RightMargin;
	const int leftMargin = img1.cols * _LeftMargin;
	const int upMargin = img1.rows * _UpMargin;
	const int downMargin = img1.rows * _DownMargin;


	Point ideal_Pt;
	float tempAngle = ideal_angle;

	// ���z���W�̎w��i�G���R�[�_��T�[�{�̎w��l�ɂ��j
	ideal_x -= leftMargin;
	ideal_y -= upMargin;

	// �摜img1����}�b�`���O�Ώۗ̈���w�肵�čĒ�`
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (img1.cols - leftMargin - rightMargin), (img1.rows - upMargin - downMargin)));

	ideal_Pt = Point(ideal_x, ideal_y);
	tempAngle = ideal_angle;
	MatchingEvaluation(fieldMap, img2, tempAngle, kakudoHaba1, kizamiKakudo1, ideal_Pt.x, ideal_Pt.y);
	// ���̕ӂŃ}�b�`���O���s��Ԃ��H

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

	std::cout << "���Γx\tx,y\n" << sp_angle << "   \t" << sp_x << "," << sp_y << std::endl;
	std::cout << "�������ԁF" << clock() - start << "[ms]" << std::endl;

	waitKey(0);
}
