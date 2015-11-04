#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>
#include <math.h>
#include <iostream>

# define PI 3.1415926

using namespace cv;

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/a108.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 1

// �O���[�o�����C���[�W�̃T�C�Y�w��
# define _FieldHeight 1000
# define _FieldWidth 1000

// �O�����}�b�`���O�ΏۂƂ��Ȃ��Ƃ��A�O������̗����w��
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

// ��]�s��ɗp����{���i1�{�ɌŒ�j
# define scale 1

float	kakudoHaba0 = 45;	// �e�X�g�����p
float	kakudoHaba1 = 24;	// 1��ڊp�x���i�Е����j
float	kakudoHaba2 = 5;	// 2���

float	kizamiKakudo0 = 5;	// �e�X�g�����p
float	kizamiKakudo1 = 3;	// 1��ڍ��݊p�x
float	kizamiKakudo2 = 1;	// 2���

// �ő�̓_�ł̊e�l��max�t���̕ϐ��Ɋi�[
Point maxPt;
Mat maxMatch;
// Eva0�̓}�b�`���O���݂̂̐��f�[�^
double	maxEvaluation0 = 0;
float	maxAngle = 0.0;
// �̗p����]���l����
float maxEvaluation = -10000;
// ���̕]�����@�ł̏o�͗p�̎Q�l�l
float maxEvaluation1 = -10000;
// �ő�̓_�ł̗��z�_�Ƃ̑��΋���
int maxDistance;


void Hyoka1(float tilt, int dist, float matchRatio, float& score){
	score = (matchRatio * 100 - dist / 5 * (cos(tilt * PI / 360) + 1));
}


void MatchingEvaluation(const cv::Mat img1,			// �O���[�o�����C���[�W�i��j
	const cv::Mat img2,			// ���[�J�����C���[�W�i���j
	float angle_center,			// ���̒��S
	float angleHalfRange,		// �p�x��
	float angleDelta			// ���݊p�x
	)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;		// �}�b�`���O���̔z��
	Mat kaitenImg;	// ��]�������n�}���i�[
	Point Pt;		// �摜���ł̍ō��}�b�`���O���̍��W���i�[
	float distance;		// ���z���W�ƃ}�b�`���O���W�̑��΋���
	float Evaluation1;	// �]���l

	maxEvaluation0 = 0;
	maxAngle = 0.0;
	maxEvaluation = -10000;
	maxEvaluation1 = -10000;
	maxDistance = 0;

	std::cout << "�@���@�����p�x�F" << (int)angle_center << "[deg]�@�s�b�`�F" << (int)angleDelta << "[deg]�@�@�}�b�`���O�J�n\n" << "��]�p\t���֒l   \tx,y\t����\t�]���l" << std::endl;

	for (float i = (angle_center)-(angleHalfRange); i < ((angle_center)+(angleHalfRange)+1); i += angleDelta)
	{
		// ��]�p[deg]
		float angle = i;

		// �摜�̒��S�����߂�
		// ��]�s��̒�`
		// �摜����]
		// ��]��摜���E�B���h�E�\��
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		warpAffine(img2, kaitenImg, matrix, img2.size());

		// �e���v���[�g�}�b�`���O
		// ���֒l�̔z�񂩂�ő�l�𒊏o����
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);

		for (int k = 0; k < match.cols; k++)
		{
			for (int j = 0; j < match.rows; j++)
			{
				if (match.at<float>(j, k) > 0.1)
				{// �}�b�`���O��0.1�𒴂����Ƃ��̂ݕ]������
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
	std::cout << "�]����F1" << "\n" << std::endl;

	// �摜�̔z���錾
	Mat img1 = imread(_ImageField);
	Mat img2 = imread(_ImageMatch);
	//Mat img1 = imread("./img/fieldMap2.jpg");
	//Mat img2 = imread("./img/c109.jpg");
	Mat kaitenImg;

	// ���z���W�̎w��i�G���R�[�_��T�[�{�̎w��l�ɂ��j
	ideal_x -= leftMargin;
	ideal_y -= upMargin;

	// �摜img1����}�b�`���O�Ώۗ̈���w�肵�čĒ�`
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (1000 - leftMargin - rightMargin), (1000 - upMargin - downMargin)));


	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba1, kizamiKakudo1);

	// ���̕ӂŃ}�b�`���O���s��Ԃ��H

	MatchingEvaluation(fieldMap, img2, maxAngle, kakudoHaba2, kizamiKakudo2);

	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, maxAngle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	rectangle(fieldMap, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);

	std::cout << "���Γx\t���֒l   \tx,y\t����\t�]���l" << std::endl;
	std::cout << (int)maxAngle << "\t" << maxEvaluation0 << "   \t" << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "\t" <<
		maxEvaluation << std::endl;
	std::cout << "�������ԁF" << clock() - start << "[ms]" << std::endl;

	imshow("Image", fieldMap);
	imshow("kaitenImg", kaitenImg);
	waitKey(0);
}

