#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

int		ideal_x = 0.0;
int		ideal_y = 0.0;
double	maxValue = 0;
float	Angle = 0.0;
float	D = 0.0;

# define _Evaluate 1
// 0:�e���v���[�g�}�b�`���O�̂�  1:�]���l�v�Z
# define _Evaluate 0 

// # define _ImageDirectory "/img
// # define _ImageField "/img/fieldMap2.jpg"
// # define _ImageMatch " ## \" ## /img/a001.jpg ## \" ## "



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

		printf("%d\t%lf\t%d,%d\n", (int)i, maxVal, Pt.x, Pt.y);

		if (maxVal > maxValue){
			ideal_x = Pt.x;
			ideal_y = Pt.y;
			maxValue = maxVal;
			Angle = angle;
		}
	}
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

		std::cout << (int)i << "\t" << maxVal << "\t" << Pt.x << "," << Pt.y << "\t" 
			<< (int)distance << "\t" << Evaluation << std::endl;

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
	}
}


int main()
{
	//Mat img1 = imread(_ImageField);
	//Mat img2 = imread(_ImageMatch);
	Mat img1 = imread("/img/fieldMap2.jpg");
	Mat img2 = imread("/img/c109.jpg");
	Mat match;	
	Mat kaitenImg;
	Point Pt;
	Point maxPt;
	float ex_angle = 0.0;
	float distance;
	double maxVal;
	double maxValue = 0;

	clock_t start = clock();

	// img1�̉摜�̗̈���w��
	Mat sub = img1(Rect(0, 0, 1000,1000));
	//imshow("Img", img2);

	int m = 3;
	printf("���Ȉʒu�������Ă��܂�...\n", m);
	Localization(sub, img2, 0, 30, m);
	printf("%f\t%lf\t%d,%d\n", Angle, maxValue, ideal_x, ideal_y);

	int n = 5;
	printf("%d�x������Ń}�b�`�J�n\n" , n);
	printf("�x\t���֒l\t\tx,y\t����\t�]���l\n");
	MatchingEvaluation(sub, img2, 0, 30, n);

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

	//MatchingEvaluation(sub, img2, Angle, 0, 1);

	float angle = Angle;
	float scale = 1.0;
	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	imshow("kaitenImg", kaitenImg);
	matchTemplate(sub, kaitenImg, match, CV_TM_CCOEFF_NORMED);
	minMaxLoc(match, NULL, &maxValue, NULL, &maxPt);
	rectangle(sub, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	printf("Angle = %f position.x = %d position.y = %d \n", angle, maxPt.x , maxPt.y);
	printf("%ld\n" , clock()-start);
	imshow("Image", sub);
	waitKey(0);
}

