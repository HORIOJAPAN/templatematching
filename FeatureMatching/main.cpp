#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>

using namespace cv;

int		ideal_x = 635;
int		ideal_y = 464;
float	Angle = 10.0;
float	D = 0.0;
double	maxValue = 0;

# define ai

void MatchingEvaluation(
	const cv::Mat img1,			// �摜�P�̃t�@�C����
	const cv::Mat img2,			// �摜�Q�̃t�@�C����
	float angle_center,			// ���̒��S
	float angle_width,			// �p�x��
	float angle_shredded		// ���݊p�x
	)
{

	Mat match;
	Mat kaitenImg;
	Point Pt;
	float distance;
	float Evaluation;
	double maxVal;
	double maxVal_1 = 0, maxVal_2 = 0;

	for (float i = (angle_center)-(angle_width); i < (angle_center) + (angle_width) + 1 ; i += angle_shredded){

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
		imshow("kaitenImg", kaitenImg);
		// �}�b�`���O
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		// ���֒l�����߂�
		minMaxLoc(match, NULL, &maxVal, NULL, &Pt);
		// �G���R�[�_�ɂ�鎩�Ȉʒu�ƃ}�b�`���O��̎��Ȉʒu�̑��΋��������߂�
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));

		if (distance > 6){
			Evaluation = maxVal - log10(distance) / 5;
		}
		else{
			Evaluation = maxVal;
		}

		printf("%d\t%lf\t%d,%d \t%d\t%lf\n", (int)i, maxVal, Pt.x, Pt.y, (long)distance, Evaluation);

# ifdef ai
		if (maxVal > maxValue){
			maxValue = maxVal;
			Angle = angle;
		}
# else
		if (Evaluation > 0){
			maxValue = maxVal;
			if (D < Evaluation){
				D = Evaluation;
				Angle = angle;
			}
		}
# endif
	}

}


int main()
{

	Mat img1 = imread("C:/Users/user/Desktop/FeatureMatching/FeatureMatching/img/fieldMap2.jpg");
	Mat img2 = imread("C:/Users/user/Desktop/FeatureMatching/FeatureMatching/img/c109.jpg");
	Mat match;	
	Mat kaitenImg;
	Point Pt;
	Point Pt2;
	Point maxPt;
	float ex_angle = 0.0;
	float distance;
	float distance_point;
	double minVal , maxVal;
	double minValue = 0 , maxValue = 0;
	double maxVal_1 = 0 , maxVal_2 = 0;
	int count = 0;

	// img1�̉摜�̗̈���w��
	Mat sub = img1(Rect(0, 0, 1000,1000));
	imshow("Img", img2);

	int n = 5 ;
	printf("%d�x������Ń}�b�`�J�n\n" , n);
	printf("�x\t���֒l\t\tx,y\t����\t�]���l\n");

	clock_t start = clock();

	//MatchingEvaluation(sub,img2,0,30,n);

# ifndef ai
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
	minMaxLoc(match, &minValue, &maxValue, NULL, &maxPt);
	
	rectangle(sub, maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	printf("Angle = %f position.x = %d position.y = %d \n", angle, maxPt.x , maxPt.y);
	printf("%ld\n" , clock()-start);
	imshow("Image", sub);
	waitKey(0);

}

