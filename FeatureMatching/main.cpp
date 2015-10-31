#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/nonfree/nonfree.hpp> // SIFT�ESURF���W���[���p
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

// # define _HyokaNumber 0 : �]���1,2,3�̂ǂ�ɂ��邩�C0�ɂ���ƃ}�b�`���O���݂̂ŏ���
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
float	kakudoHaba1 = 18;	// 1��ڊp�x���i�Е����j
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
float maxEvaluation2 = -10000;
float maxEvaluation3 = -10000;
// �ő�̓_�ł̗��z�_�Ƃ̑��΋���
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
 * �����́utilt:���Ίp�x�v�udist:���΋����v�umatchRatio:�摜�̃}�b�`���O���v�uscore:�]���l��Ԃ��|�C���^�v
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
	score = 100 - dist / matchRatio;
}
void Hyoka3(float tilt, int dist, float matchRatio, float& score){
	score = 100 - pow(dist,0.7) / matchRatio * (cos(tilt * PI / 360) + 1) / 2;
}


void MatchingEvaluation(	const cv::Mat img1,			// �O���[�o�����C���[�W�i��j
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
	double Evaluation0;	// ���}�b�`���O���̍ő�l
	float Evaluation1;	// �]���l
	float Evaluation2;	// �]���l
	float Evaluation3;	// �]���l

	// printf("%d�x������Ń}�b�`�J�n\n", (int)angleDelta);
	// printf("���Γx\t���֒l   \tx,y\t����\t�]���l\n");
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
		imshow("kaitenImg", kaitenImg);

		// �e���v���[�g�}�b�`���O
		// ���֒l�̔z�񂩂�ő�l�𒊏o����
		matchTemplate(img1, kaitenImg, match, CV_TM_CCOEFF_NORMED);
		minMaxLoc(match, NULL, &Evaluation0, NULL, &Pt);

		// ���z�̎��Ȉʒu���W�ƍœK�}�b�`���O���W�Ƃ̑��΋����̎Z�o
		distance = sqrt(powf((Pt.x - ideal_x), 2) + powf((Pt.y - ideal_y), 2));

		// �e��]���l�̌v�Z
		Hyoka1(angle, distance, Evaluation0, Evaluation1);
		Hyoka2(angle, distance, Evaluation0, Evaluation2);
		Hyoka3(angle, distance, Evaluation0, Evaluation3);
		// �imatch�z��̑S�v�f�ɑ΂��ĕ]���������ׂ��j

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
	std::cout << "�]����F" << _HyokaNumber << "\n" << std::endl;

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


	if (!_HyokaNumber){
		printf("�}�b�`���O�����玩�Ȉʒu�������Ă��܂�...\n", kizamiKakudo0);
		MatchingEvaluation(fieldMap, img2, 0, kakudoHaba0, kizamiKakudo0);
	}

	MatchingEvaluation(fieldMap, img2, 0, kakudoHaba1, kizamiKakudo1);

	/* �}�b�`���O���s��Ԃ�
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


	std::cout << "���Γx\t���֒l   \tx,y\t����\t�]���l" << std::endl;
	std::cout << (int)maxAngle << "\t" << maxEvaluation0 << "   \t" << maxPt.x << "," << maxPt.y << "\t" << (int)maxDistance << "\t" <<
		maxEvaluation1 << "   \t" << maxEvaluation2 << "   \t" << maxEvaluation3 << std::endl;
	// printf("%ld[ms]\n" , clock()-start);
	std::cout << "�������ԁF" << clock() - start << "[ms]" << std::endl;

	imshow("Image", fieldMap);
	waitKey(0);
}

