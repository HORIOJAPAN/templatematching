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


void Hyoka1(float tilt, float dist, float matchRatio, float& score){
	score = (matchRatio * 100 - dist / 5 * (cos(tilt * 3.1415926 / 360) + 1));
}

void MatchingEvaluation(
	const cv::Mat img1,			// �O���[�o�����C���[�W�i��j
	const cv::Mat img2,			// ���[�J�����C���[�W�i���j
	float angle_base,			// �]����p�x
	float &angle_center,		// �]�����S�p�x
	float angleHalfRange,		// �͈͔��p
	float angleDelta,			// ���݊p�x
	float ideal_x,
	float ideal_y
	)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;		// �}�b�`���O���̔z��
	Mat kaitenImg;	// ��]�������n�}���i�[
	Point Pt;		// �摜���ł̍ō��}�b�`���O���̍��W���i�[
	float distance;		// ���z���W�ƃ}�b�`���O���W�̑��΋���
	float Evaluation1;	// �]���l

	double maxEvaluation0 = 0;
	float maxAngle = 0;
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

		for (float k = 0; k < match.cols; k++)
		{
			for (float j = 0; j < match.rows; j++)
			{
				distance = sqrt((k - ideal_x)*(k - ideal_x) + (j - ideal_y)*(j - ideal_y));
				Hyoka1((angle-angle_base), distance, match.at<float>(j, k), Evaluation1);

				if (Evaluation1 > maxEvaluation){
					maxEvaluation0 = match.at<float>(j, k);
					maxAngle = angle;
					Pt = Point(k, j);
					maxDistance = distance;
					maxEvaluation = Evaluation1;
				}
			}
		}
		std::cout << (int)angle << "\t" << maxEvaluation0 << "   \t" << Pt.x << "," << Pt.y << "\t" << maxDistance << "\t" << maxEvaluation << std::endl;
	}
	std::cout << std::endl;

	sp_x = Pt.x;
	sp_y = Pt.y;
	sp_angle = maxAngle;
	angle_center = sp_angle;
}

# define fieldSquareSize 350		// �g���~���O��A�t�B�[���h�摜�̏c���t������
# define matchSquareSize 200		// �g���~���O��A�}�b�`�摜�̏c���t������

# define kakudoHaba1 24		// 1��ڊp�x���i�Е����j
# define kakudoHaba2 5		// 2���

# define kizamiKakudo1 3	// 1��ڍ��݊p�x
# define kizamiKakudo2 1	// 2���

void spEstimate(int ideal_x, int ideal_y , float ideal_angle , Mat img1, Mat img2)
{ // Self Position Estimate		//(x, y, angle, fieldMap, matchMap)

	// �摜�̔z���錾
	// Mat img1 = imread(imageName);
	// Mat img2 = imread("./img/a001.jpg");

	const int leftBorder = ideal_x - (fieldSquareSize - matchSquareSize) / 2;
	const int upBorder = ideal_y - (fieldSquareSize - matchSquareSize) / 2;
	const int rightBorder = ideal_x + (fieldSquareSize + matchSquareSize) / 2;
	const int downBorder = ideal_y + (fieldSquareSize + matchSquareSize) / 2;

	const int leftBorder2 = (img2.cols - matchSquareSize)/ 2;
	const int upBorder2 = (img2.rows - matchSquareSize) / 2;
	const int rightBorder2 = (img2.cols + matchSquareSize) / 2;
	const int downBorder2 = (img2.rows + matchSquareSize) / 2;

	// �摜img1����}�b�`���O�Ώۗ̈���w�肵�čĒ�`
	Mat fieldMap = img1(Rect(leftBorder, upBorder, rightBorder - leftBorder, downBorder - upBorder));
	Mat matchMap = img2(Rect(leftBorder2, upBorder2, rightBorder2 - leftBorder2, downBorder2 - upBorder2));
	// Mat matchMap = img2(Rect(leftBorder, upBorder, rightBorder - leftBorder, downBorder - upBorder));

	std::cout << "���}�b�`�摜�T�C�Y\t" << img2.rows << "\t" << img2.cols << "\t" << "�V�}�b�`�摜�T�C�Y\t" << matchMap.cols << "\t" << matchMap.rows << "\t" << "\n" << std::endl;

	Point ideal_Pt;
	ideal_Pt = Point(ideal_x, ideal_y);
	float tempAngle = ideal_angle;	// �]���̒��S�p�̏�����

	// ���z���W�̎w��i�G���R�[�_��T�[�{�̎w��l�ɂ��j
	ideal_Pt.x -= leftBorder;
	ideal_Pt.y -= upBorder;


	MatchingEvaluation(fieldMap, matchMap, ideal_angle, tempAngle, kakudoHaba1, kizamiKakudo1, ideal_Pt.x, ideal_Pt.y);
	// ���̕ӂŃ}�b�`���O���s��Ԃ��H

	MatchingEvaluation(fieldMap, matchMap, ideal_angle, tempAngle, kakudoHaba2, kizamiKakudo2, ideal_Pt.x, ideal_Pt.y);
	
	Point2f center(matchMap.cols / 2.0, matchMap.rows / 2.0);
	Mat kaitenImg;
	Mat matrix = cv::getRotationMatrix2D(center, tempAngle, 1);
	warpAffine(matchMap, kaitenImg, matrix, matchMap.size());
	rectangle(fieldMap, Point(sp_x, sp_y), Point(sp_x + kaitenImg.cols, sp_y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);

	imshow("Image", fieldMap);
	imshow("kaitenImg", kaitenImg);
}

// _MatchArea => a:1 b:2 c:3
# define _MatchArea 1

int ideal_angle = -8;

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
	Mat img1 = imread("./img/fieldMap2.jpg");
	Mat img2 = imread("./img/a001.jpg");

	spEstimate(sp_x, sp_y, sp_angle, img1, img2);

	// ������̌��ʂ�sp_angle���}�b�`���O���ʂ̍��W�Ɗp�x�̐�Βl(�t�B�[���h�摜�̍��W�n)
	sp_x += ideal_x - (fieldSquareSize - matchSquareSize) / 2;
	sp_y += ideal_y - (fieldSquareSize - matchSquareSize) / 2;

	std::cout << "���́F\n���Γx\tx,y\n" << ideal_angle << "   \t" << ideal_x << "," << ideal_y << std::endl;
	std::cout << "�o�́F\n���Γx\tx,y\n" << sp_angle << "   \t" << sp_x << "," << sp_y << std::endl;
	std::cout << "�������ԁF" << clock() - start << "[ms]" << std::endl;

	waitKey(0);
}
