#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

# define _ImageField "./img/fieldMap2.jpg"
# define _ImageMatch "./img/b102.jpg"
// _MatchArea => a:1 b:2 c:3
# define _MatchArea 2

// _HyokaKizyun : �]���1,2,3�̂ǂ�ɂ��邩�C0�ɂ���ƃ}�b�`���O���݂̂ŏ���
# define _HyokaKizyun Evaluation1

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

// �ő�̓_�ł̊e�l��max�t���̕ϐ��Ɋi�[
double	maxValue = 0;
float	maxAngle = 0.0;

float	kakudoHaba0 = 45;	// �e�X�g�����p
float	kakudoHaba1 = 24;	// 1��ڊp�x���i�Е����j
float	kakudoHaba2 = 8;	// 2���

float	kizamiKakudo0 = 5;	// �e�X�g�����p
float	kizamiKakudo1 = 4;	// 1��ڍ��݊p�x
float	kizamiKakudo2 = 1;	// 2���

Point maxPt;
Mat maxMatch;
float maxEvaluation = -100;
float maxEvaluation1 = -100;
float maxEvaluation2 = -100;
float maxEvaluation3 = -100;
int maxDistance;


/*
 * �ύX����ӏ�
 * 
# define _ImageField "./img/fieldMap2.jpg"
 * ��fieldMap.jpg �� fieldMap2.jpg ��҂��菑���ŏC���������́D��{�I�Ɍ�҂��g��
 * 
# define _ImageMatch "./img/a002.jpg"
 * ��a,b,c 001~ , 101~ �������C�f�B���N�g���ɂȂ��t�@�C�����Q�Ƃ���ƃG���[�ɂȂ�̂ł悭����
 * 
# define _MatchArea 1
 * ����ŕҏW�����}�b�`���O�摜�̓������ɉ����ĕύX����D���z���W��ύX����i���Ȉʒu�̊���W�j
 * a:1 b:2 c:3
 * 
 */

/*
 * �]���l�̌v�Z���@
 * �����́utilt:���Ίp�x�v�udist:���΋����v�umatchRatio:�摜�̃}�b�`���O���v
 * �p�x��[deg]�C������[pixel/5cm],�}�b�`���O���� 1 ~ 0
 * �p�x�ƍ��W�͎��ۂ̃��{�b�g�̈ړ����x���l������
 * �}�b�`���O����2���̉摜���ڂ����Ƃ��ɂ��̃s�N�Z�����ƂɃA�����i�C���𔻒肵�Ă�l�q
 * ����ɂ���āC���摜�̐��������Ƒ����̊p�x�͋��e�����Ȃ�
 * Hyoka1�ł̓}�b�`���O���Ɍ������킹�Č��_�@�Ő����𔻒f���Ă��邪�C����ɔ�����K�v�͂Ȃ�
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
	score = matchRatio * 100 - dist;
}
void Hyoka3(float tilt, int dist, float matchRatio, float& score){
	score = 100 - dist - abs(tilt)*3 ;
}


void MatchingEvaluation(	const cv::Mat img1,			// �摜�P�̃t�@�C����
							const cv::Mat img2,			// �摜�Q�̃t�@�C����
							float angle_center,			// ���̒��S
							float angleHalfRange,		// �p�x��
							float angleDelta			// ���݊p�x
							)
{
	// �e���v���[�g�}�b�`���O�ƕ]���l�̎Z�o�E�œK�l�̏o��

	Mat match;		// �}�b�`���O���̔z��
	Mat kaitenImg;
	Point Pt;		// �摜���ł̍ō��}�b�`���O���̍��W���i�[
	float distance;		// ���z���W�ƃ}�b�`���O���W�̑��΋���
	float Evaluation1;	// �]���l
	float Evaluation2;	// �]���l
	float Evaluation3;	// �]���l
	double Evaluation0;		// �}�b�`���O���̍ő�l
	// double maxVal_1 = 0, maxVal_2 = 0;

	for (float i = (angle_center)-(angleHalfRange); i < ((angle_center)+(angleHalfRange)+1); i += angleDelta){
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
		minMaxLoc(match, NULL, &Evaluation0, NULL, &Pt);
		// �G���R�[�_�ɂ�鎩�Ȉʒu�ƃ}�b�`���O��̎��Ȉʒu�̑��΋��������߂�
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));


		Hyoka1(angle, distance, Evaluation0, Evaluation1);
		Hyoka2(angle, distance, Evaluation0, Evaluation2);
		Hyoka3(angle, distance, Evaluation0, Evaluation3);


		if (_HyokaKizyun > maxEvaluation){
			maxValue = Evaluation0;
			maxAngle = angle;
			maxPt = Pt;
			maxMatch = match;
			maxDistance = distance;
			maxEvaluation = _HyokaKizyun;
			//maxEvaluation1 = Evaluation1;
			//maxEvaluation2 = Evaluation2;
			//maxEvaluation3 = Evaluation3;
		}

		std::cout << (int)i << "\t" << Evaluation0<< "   " << Pt.x<< "," << Pt.y << "   " << (int)distance << "\t" <<
			Evaluation1 << "\t" << Evaluation2 << "\t" << Evaluation3 << std::endl;
	}
}


int main()
{
	clock_t start = clock();

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
	Mat fieldMap = img1(Rect(leftMargin, upMargin, (1000 - leftMargin - rightMargin), (1000 - upMargin - downMargin)) );

# if _Evaluate
	// 
	int m = 5;
	printf("���Ȉʒu�������Ă��܂�...\n", m);
	// Localization(fieldMap, img2, 0, 45, m);
	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba0, kizamiKakudo0);

# endif

	printf("%d�x������Ń}�b�`�J�n\n", (int)kizamiKakudo1);
	printf("���Γx\t���֒l\t   x,y\t   ����\t�]���l\n");
	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba1, kizamiKakudo1);

	/*
	if (D == 0.0){
		printf("%f\n", D);
		printf("No matching\n");
		return(0);
	}
	*/

	printf("%d�x������Ń}�b�`�J�n\n", (int)kizamiKakudo2);
	printf("���Γx\t���֒l\t  x,y\t   ����\t�]���l\n");

	MatchingEvaluation(fieldMap, img2, maxAngle, kakudoHaba2, kizamiKakudo2);

	float angle = maxAngle;
	float scale = 1.0;
	Point2f center(img2.cols / 2.0, img2.rows / 2.0);
	Mat matrix = cv::getRotationMatrix2D(center, angle, scale);
	warpAffine(img2, kaitenImg, matrix, img2.size());
	rectangle(fieldMap , maxPt, Point(maxPt.x + kaitenImg.cols, maxPt.y + kaitenImg.rows), Scalar(0, 0, 255), 2, 8, 0);
	imshow("kaitenImg", kaitenImg);

	std::cout << "\n" << (int)angle << "\t" << maxValue << "    " << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "   " <<
		maxEvaluation1 << "\t" << maxEvaluation2 << "\t" << maxEvaluation3 << std::endl;
	printf("%ld[ms]\n" , clock()-start);

	imshow("Image", fieldMap);
	waitKey(0);
}

