#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

# define _Evaluate 0
// 0:���Ȉʒu�𒼐ړ���  1:���֒l�݂̂Ŏ��Ȉʒu�𐄒�
// 0:�e���v���[�g�}�b�`���O�̂�  1:�]���l�v�Z

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/a002.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 1

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

double	maxValue = 0;
float	Angle = 0.0;
float	D = 0.0;



void Localization(		const cv::Mat img1,			// �摜�P�̃t�@�C����
						const cv::Mat img2,			// �摜�Q�̃t�@�C����
						float angle_center,			// ���̒��S
						float angle_width,			// �p�x��
						float angle_shredded		// ���݊p�x				
						)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;
	Mat kaitenImg;
	Point Pt;
	double maxVal;		// �}�b�`���O���̍ő�l

	for (float i = (angle_center)-(angle_width); i < ((angle_center) + (angle_width) + 1) ; i += angle_shredded){
		// ��]�F  [deg]
		float angle = i;
		// �傫���F  [�{]
		float scale = 1.0;
		// �摜�̒��S�����߂�
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		// ��]�s��
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		// �摜����]
		warpAffine(img2, kaitenImg, matrix, img2.size());
		// �E�B���h�E�\��
		imshow("kaitenImg", kaitenImg);
		// �}�b�`���O
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// ���֒l�����߂�
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

void MatchingEvaluation(	const cv::Mat img1,			// �摜�P�̃t�@�C����
							const cv::Mat img2,			// �摜�Q�̃t�@�C����
							float angle_center,			// ���̒��S
							float angle_width,			// �p�x��
							float angle_shredded		// ���݊p�x				
							)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;
	Mat kaitenImg;
	Point Pt;
	float distance;		// ���z���W�ƃ}�b�`���O���W�̑��΋���
	float Evaluation;	// �]���l
	double maxVal;		// �}�b�`���O���̍ő�l
	// double maxVal_1 = 0, maxVal_2 = 0;

	for (float i = (angle_center)-(angle_width); i < ((angle_center)+(angle_width)+1); i += angle_shredded){
		// ��]�F  [deg]
		float angle = i;
		// �傫���F  [�{]
		float scale = 1.0;
		// �摜�̒��S�����߂�
		Point2f center(img2.cols / 2.0, img2.rows / 2.0);
		// ��]�s��
		Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
		// �摜����]
		warpAffine(img2, kaitenImg, matrix, img2.size());
		// �E�B���h�E�\��
		imshow("kaitenImg", kaitenImg);
		// �e���v���[�g�}�b�`���O
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// ���֒l�����߂�
		minMaxLoc(match, NULL, &maxVal, NULL, &Pt);
		// �G���R�[�_�ɂ�鎩�Ȉʒu�ƃ}�b�`���O��̎��Ȉʒu�̑��΋��������߂�
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));


		// ���]���l�̌v�Z�͂���
		if (distance > 6){ // ������6�i6�s�N�Z���F30cm�j�����̂Ƃ�
			Evaluation = maxVal - log10(distance) / 5;
		}
		else{
			Evaluation = maxVal;
		}
		// ���]���l�̌v�Z�����

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

	// img1�̉摜�̗̈���w��
	Mat sub = img1(Rect(leftMargin, upMargin, 1000 - leftMargin - rightMargin, 1000 - upMargin - downMargin));
	//imshow("Img", img2);

# if _Evaluate

	int m = 5;
	printf("���Ȉʒu�������Ă��܂�...\n", m);
	Localization(sub, img2, 0, 45, m);

# endif

	int n = 5;
	printf("%d�x������Ń}�b�`�J�n\n" , n);
	printf("�x\t���֒l\t\tx,y\t����\t�]���l\n");
	MatchingEvaluation(sub, img2, 0, 45, n);

# if _Evaluate
	if (D == 0.0){
		printf("%f\n", D);
		printf("No matching\n");
		return(0);
	}
# endif

	printf("1�x������Ń}�b�`�J�n\n");
	printf("�x\t���֒l\t\tx,y\t\t����\t�]���l\n");

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

