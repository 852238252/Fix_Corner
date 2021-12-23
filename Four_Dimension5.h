//******************************
//      ��ά�����ǵ��Ż�
//******************************
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;
using namespace cv;


class fixCornerfourDimension
{
	public:
		Mat readProcess(Mat src);																		//Ԥ����ͼ��(ת��Ϊ����ͼ)
		bool levelLine(Mat cannyImg, Point start, Point& end, int direction, double minLen);			//���õ��Ƿ���ˮƽ������
		bool verticalLine(Mat cannyImg, Point start, Point& end, int direction, double minLen);			//���õ��Ƿ��ڴ�ֱ������
		bool fixMiddleDownCorner(Mat img, int type, Point& corner);										//�����ά�������½ǵ�
		bool fixMiddleCorner(Mat cannyImg, Point middleDownCorner, vector<Point>& cornerVec);			//�����ά�������������ǵ�
		bool Line(Mat cannyImg, Point start, int direction, int minLen);								//����߶�
		bool rightLine(Mat cannyImg, Point start, int direction, int minLen);							//����ҷ����߶�
		bool rankCorner(vector<Point>& corner, int cameraType);											//����ǵ�˳��
		bool fixCorner(Mat src, int cameraType, vector<Point>& corner);									//�����ά�����еĽǵ�(�����������)
		
};

//����:ͼ��·��;  ����:Ԥ����֮���ͼ��; �˺������ڼ��ͼ������������ں��������ͽǵ�ļ��
Mat fixCornerfourDimension::readProcess(Mat src)
{
	//�ԻҶ�ģʽ��ȡͼƬ
	Mat img = src.clone();

	//��˹�˲�
	Mat img_gauss;
	GaussianBlur(img, img_gauss, Size(9, 9), 0, 0);

	//Canny�������ͼ
	Mat img_canny;
	Canny(img_gauss, img_canny, 100, 250, 3, false);

	return img_canny;
}

//����:ͼ��������ꡢ�յ�����(���)������(0��1��)����С���ȣ�����:����Ƿ���ڴ�ֱ����ı߳����Ҹñ߳���С����ΪminLen
bool fixCornerfourDimension::levelLine(Mat cannyImg, Point start, Point& end, int direction, double minLen)
{
	int breakPoint = 0;			//���尵������
	end = start;				//��ʼ���յ�����
	double len = 0.0;			//��ʼ������

	if (direction == 0)	//����
	{
		while (breakPoint < 10)	//������������������5ʱ
		{
			//���Ʒ�Χ
			if (end.x < 20 || end.x >cannyImg.cols - 20) return false;
			if (end.y <20 || end.y >cannyImg.rows - 20) return false;

			len = start.x - end.x;	//ˮ������Ҫ��x����
			if (len >= minLen) { return true; }	//������������С���룬�򷵻���

			//�������ƶ�
			int i = 0;
			for (; i < 4; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y - i, end.x - 1) > 250)
				{
					end.y = end.y - i;
					end.x--;
					breakPoint = 0;
					break;
				}
			}

			if (i == 4) i = 0;
			else continue;

			//�������ƶ�
			for (; i < 4; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y + i, end.x - 1) > 250)
				{
					end.y = end.y + i;
					end.x--;
					breakPoint = 0;
					break;
				}
			}
			//������¶�û�����㣬�򰵵�������һ
			if (i == 4) { breakPoint++; end.x--; }
		}
	}
	else if (direction == 1)	//�ҷ���
	{
		while (breakPoint < 10)	//������������������5ʱ
		{
			//���Ʒ�Χ
			if (end.x < 20 || end.x >cannyImg.cols - 20) return false;
			if (end.y <20 || end.y >cannyImg.rows - 20) return false;

			len = end.x - start.x;	//��ֱ������Ҫ��x����
			if (len >= minLen) { return true; }	//������������С���룬�򷵻���

			//�������ƶ�
			int i = 0;
			for (; i < 4; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y + i, end.x + 1) > 250)
				{
					end.y = end.y + i;
					end.x++;
					breakPoint = 0;
					break;
				}
			}

			if (i == 4) i = 0;
			else continue;

			//�������ƶ�
			for (; i < 4; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y - i, end.x + 1) > 250)
				{
					end.y--;
					end.x++;
					breakPoint = 0;
					break;
				}
			}

			//������¶�û�����㣬�򰵵�������һ
			if (i == 4) { breakPoint++; end.x++; }
		}
	}
	return false;
}

//����:ͼ��������ꡢ�յ�����(���)������(0��1��)����С���ȣ�����:����Ƿ���ڴ�ֱ����ı߳����Ҹñ߳���С����ΪminLen
bool fixCornerfourDimension::verticalLine(Mat cannyImg, Point start, Point& end, int direction, double minLen)
{
	int breakPoint = 0;			//���尵������
	end = start;				//��ʼ���յ�����
	double len = 0.0;			//��ʼ������

	if (direction == 0)	//�Ϸ���
	{
		while (breakPoint < 10)	//������������������5ʱ
		{
			//���Ʒ�Χ
			if (end.x < 20 || end.x >cannyImg.cols - 20) return false;
			if (end.y <20 || end.y >cannyImg.rows - 20) return false;

			len = start.y - end.y;	//��ֱ������Ҫ��y����
			if (len >= minLen) { return true; }	//������������С���룬�򷵻���

			//�������ƶ�
			int i = 0;	
			for (; i < 6; i++)	
			{
				if ((int)cannyImg.at<uchar>(end.y - 1, end.x - i) > 250) 
				{
					end.y--; 
					end.x = end.x - i;
					breakPoint = 0;
					break; 
				}
			}

			if (i == 6) i = 0;
			else continue;

			//�������ƶ�
			for(; i < 6; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y - 1, end.x + i) > 250)
				{
					end.y--;
					end.x = end.x + i;
					breakPoint = 0;
					break;
				}
			}

			//������Ҷ�û�����㣬�򰵵�������һ
			if (i == 6)
			{
				breakPoint++;
				end.y--;
			}
		}
	}
	else if (direction == 1)	//�·���
	{
		while (breakPoint < 10)	//������������������5ʱ
		{
			//���Ʒ�Χ
			if (end.x < 20 || end.x >cannyImg.cols - 20) return false;
			if (end.y <20 || end.y >cannyImg.rows - 20) return false;

			len = end.y - start.y;	//��ֱ������Ҫ��y����
			if (len >= minLen) { return true; }	//������������С���룬�򷵻���

			//�������ƶ�
			int i = 0;
			for (; i < 6; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y + 1, end.x + i) > 250)
				{
					end.y++;
					end.x = end.x + i;
					breakPoint = 0;
					break;
				}
			}

			if (i == 6) i = 0;
			else continue;

			//�������ƶ�
			for (; i < 6; i++)
			{
				if ((int)cannyImg.at<uchar>(end.y + 1, end.x - i) > 250)
				{
					end.y++;
					end.x = end.x - i;
					breakPoint = 0;
					break;
				}
			}

			//������Ҷ�û�����㣬�򰵵�������һ
			if (i == 6) breakPoint++;
		}
	}
	return false;
}

//����:ͼ��; ����:��ά���������Ľǵ�����
bool fixCornerfourDimension::fixMiddleDownCorner(Mat cannyImg, int cameraType, Point& corner)
{
	//�����ⷶΧ
	int minRow = cannyImg.rows * 3 / 8, maxRow = cannyImg.rows * 7 / 8;
	int minCols, maxCols;

	//��������ͷ�����ü�ⷶΧ0~3�ֱ�Ϊǰ�����ҡ�������ͷ��
	if (cameraType == 0 || cameraType == 3)
	{
		minCols = cannyImg.cols * 3 / 8;
		maxCols = cannyImg.cols * 5 / 8;
	}
	else if (cameraType == 1)
	{
		minCols = cannyImg.cols * 4 / 8 + 20;
		maxCols = cannyImg.cols * 5 / 8 + 100;
	}
	else if (cameraType == 2)
	{
		minCols = cannyImg.cols * 2 / 8 + 60;
		maxCols = cannyImg.cols * 4 / 8 + 20;
	}
	else return false;

	//��ʼ���ǵ�
	bool sign = false;
	for (int row = maxRow; row > minRow; row--)
	{
		for (int cols = minCols; cols < maxCols; cols++)
		{
			//���ǰ���������
			if ((int)cannyImg.at<uchar>(row, cols) < 250) continue;

			//���Ϸ�������������
			int topLen = 0;
			for (; topLen < 10; topLen++)
			{
				if ((int)cannyImg.at<uchar>(row - 10, cols + topLen) > 250) break;
				if ((int)cannyImg.at<uchar>(row - 10, cols - topLen) > 250) { topLen = -topLen; break; }

			}
			if (topLen == 10)continue;

			//���·�������������
			int downLen = 0;
			for (; downLen < 10; downLen++)
			{
				if ((int)cannyImg.at<uchar>(row + 10, cols + downLen) > 250) break;
				if ((int)cannyImg.at<uchar>(row + 10, cols - downLen) > 250) { downLen = -downLen; break; }
			}
			if (downLen == 10)continue;

			//����������������
			int leftLen = 0;
			for (; leftLen < 10; leftLen++)
			{
				if ((int)cannyImg.at<uchar>(row + leftLen, cols - 20) > 250) break;
				if ((int)cannyImg.at<uchar>(row - leftLen, cols - 20) > 250) { leftLen = -leftLen; break; }
			}
			if (leftLen == 10)continue;

			//���ҷ�������������
			int rightLen = 0;
			for (; rightLen < 10; rightLen++)
			{
				if ((int)cannyImg.at<uchar>(row + rightLen, cols + 20) > 250) break;
				if ((int)cannyImg.at<uchar>(row - rightLen, cols + 20) > 250) { rightLen = -rightLen; break; }
			}
			if (rightLen == 10)continue;

			//����Ƿ���������߳�
			Point topEnd;
			if (!(verticalLine(cannyImg, Point(cols + topLen, row - 10), topEnd, 0, 100))) continue;
			Point downEnd;
			if (!(verticalLine(cannyImg, Point(cols + downLen, row + 10), topEnd, 1, 50))) continue;
			Point leftEnd;
			if (!(levelLine(cannyImg, Point(cols - 20, row + leftLen), leftEnd, 0, 100))) continue;
			Point rightEnd;
			if (!(levelLine(cannyImg, Point(cols + 20, row + rightLen), rightEnd, 1, 100))) continue;

			corner = Point(cols, row);
			sign = true;
			break;
		}
		if (sign) break;
	}

	if (!sign) return false;
	vector<Point> middleCornerVec;		//�����м������ǵ�
	if (!fixMiddleCorner(cannyImg, corner, middleCornerVec)) return false;

	//�ҳ���ǵ�
	Point leftPoint;
	int row = corner.y;
	int cols = corner.x;
	int darkNum = 0;
	//������
	for (int leftLen = 0; leftLen < 30; leftLen++)
	{
		if ((int)cannyImg.at<uchar>(row + leftLen, cols - 30) > 250) { row = row + leftLen; cols = cols - 30; break; }
		if ((int)cannyImg.at<uchar>(row - leftLen, cols - 30) > 250) { row = row - leftLen; cols = cols - 30; break; }
	}
	while (1)
	{
		bool sign = false;
		if (cols <= 0 || row <= 0 || row >= cannyImg.rows) return false;
		int move = 0;
		for (; move < 8; move++)
		{
			if ((int)cannyImg.at<uchar>(row - move, cols - 1))break;
			if ((int)cannyImg.at<uchar>(row + move, cols - 1)) { move = -move; break; }
		}
		if (move < 8)
		{
			row = row - move;
		}
		else
		{
			darkNum++;
			if (darkNum > 40) return false;
		}
		cols--;
		if (corner.x - cols < 100) continue;

		for (int top = 0; top < 8; top++)
		{
			if ((int)cannyImg.at<uchar>(row - 10, cols-top) > 250)
			{
				Point end;
				if (verticalLine(cannyImg, Point(cols - top, row - 10), end, 0, 20))
				{
					leftPoint = Point(cols - top, row);
					sign = true;
					break;
				}
			}
			if ((int)cannyImg.at<uchar>(row - 10, cols + top) > 250)
			{
				Point end;
				if (verticalLine(cannyImg, Point(cols + top, row - 10), end, 0, 20))
				{
					leftPoint = Point(cols - top, row);
					sign = true;
					break;
				}
			}

		}
		if (sign) break;
	}

	//�ҳ��ҽǵ�
	Point rightPoint;
	row = corner.y;
	cols = corner.x;
	darkNum = 0;

	//����ҵ�
	for (int leftLen = 0; leftLen < 30; leftLen++)
	{
		if ((int)cannyImg.at<uchar>(row + leftLen, cols + 30) > 250) { row = row + leftLen; cols = cols + 30; break; }
		if ((int)cannyImg.at<uchar>(row - leftLen, cols + 30) > 250) { row = row - leftLen; cols = cols + 30; break; }
	}
	while (1)
	{
		bool sign = false;
		if (cols >= cannyImg.cols || row <= 0 || row >= cannyImg.rows) return false;
		int move = 0;
		for (; move < 8; move++)
		{
			if ((int)cannyImg.at<uchar>(row - move, cols + 1))break;
			if ((int)cannyImg.at<uchar>(row + move, cols + 1)) { move = -move; break; }
		}
		if (move < 8)
		{
			row = row - move;
		}
		else
		{
			darkNum++;
			if (darkNum > 40) return false;
		}
		cols++;

		if (cols - corner.x < 100) continue;
		for (int top = 0; top < 8; top++)
		{
			if ((int)cannyImg.at<uchar>(row - 10, cols - top) > 250)
			{
				Point end;
				if (verticalLine(cannyImg, Point(cols - top, row - 10), end, 0, 20))
				{
					rightPoint = Point(cols + top, row);
					sign = true;
					break;
				}
			}
			if ((int)cannyImg.at<uchar>(row - 10, cols + top) > 250)
			{
				Point end;
				if (verticalLine(cannyImg, Point(cols + top, row - 10), end, 0, 20))
				{
					rightPoint = Point(cols + top, row);
					sign = true;
					break;
				}
			}
		}
		if (sign) break;
	}

	bool leftsign = false, rightsign = false;
	cols = (corner.x + leftPoint.x) / 2;
	if (corner.y < leftPoint.y)
	{
		row = corner.y - 5;
	}
	else
	{
		row = leftPoint.y - 5;
	}
	int mRow = (middleCornerVec[0].y + middleCornerVec[1].y) / 2 - 40;
	while (row > mRow)
	{ 
		for (int len = 0; len < 8; len++)
		{
			if ((int)cannyImg.at<uchar>(row, cols - len) > 250 || (int)cannyImg.at<uchar>(row, cols + len) > 250) { leftsign = true; break; }
		}
		row--;
	}


	cols = (corner.x + rightPoint.x) / 2;
	if (corner.y < rightPoint.y) row = corner.y - 5;
	else row = rightPoint.y - 5;
	while (row > mRow)
	{
		for (int len = 0; len < 8; len++)
		{
			if ((int)cannyImg.at<uchar>(row, cols - len) > 250 || (int)cannyImg.at<uchar>(row, cols + len) > 250) { rightsign = true; break; }
		}
		row--;
	}

	if (leftsign && rightsign) return true;
	else if (!leftsign && rightsign) corner = rightPoint;
	else if (leftsign && !rightsign) corner = leftPoint;

	return true;
}

//����:ͼ�����½ǵ����Χ������; ����:�ǵ���������
bool fixCornerfourDimension::fixMiddleCorner(Mat cannyImg, Point middleDownCorner, vector<Point>& cornerVec)
{
	//��ȡ���½ǵ������
	int row = middleDownCorner.y;
	int cols = middleDownCorner.x;

	for (int i = 0; i < 10; i++)
	{
		if ((int)cannyImg.at<uchar>(row - 10, cols + i) > 250)
		{
			row = row - 10;
			cols = cols + i;
			break;
		}
		if ((int)cannyImg.at<uchar>(row - 10, cols - i) > 250)
		{
			row = row - 10;
			cols = cols - i;
			break;
		}
	}

	//���÷�Χ
	int minRow = 20, maxRow = cannyImg.rows - 20;
	int minCols = cannyImg.cols * 2 / 8, maxCols = cannyImg.cols * 6 / 8;

	//��ʼ����������
	int darkPoint = 0;
	int len = 0;
	Point lastPoint(middleDownCorner);
	cornerVec.push_back(middleDownCorner);

	//��ʼ����м�Ľǵ�(2��)
	while (row>minRow && row < maxRow && cols > minCols && cols <maxCols)
	{
		//�����ƶ���������
		int i = 0;
		for (; i < 5; i++)
		{
			if ((int)cannyImg.at<uchar>(row - 1, cols - i) > 250)
			{
				row--;
				cols = cols - i;
				darkPoint = 0;
				break;
			}
			else if ((int)cannyImg.at<uchar>(row - 1, cols + i) > 250)
			{
				row--;
				cols = cols + i;
				darkPoint = 0;
				break;
			}
		}

		//���Ҳ������㣬�򰵵�����+1
		if (i == 5) 
		{ 
			darkPoint++; 
			row--; 
			if (darkPoint > 10) return false;
		}

		//������ֱ����
		len = lastPoint.y - row;
		if (len > 50)
		{
			int leftLen = 0;
			for (; leftLen < 8; leftLen++)
			{
				if ((int)cannyImg.at<uchar>(row - leftLen, cols - 10) > 250) break;
				if ((int)cannyImg.at<uchar>(row + leftLen, cols - 10) > 250) { leftLen = -leftLen; break;}

			}
			if (leftLen == 8) continue;

			int rightLen = 0;
			for (; rightLen < 8; rightLen++)
			{
				if ((int)cannyImg.at<uchar>(row - rightLen, cols + 10) > 250) break;
				if ((int)cannyImg.at<uchar>(row + rightLen, cols + 10) > 250) { rightLen = -rightLen; break; }
			}
			if (rightLen == 8) continue;

			Point leftEnd;
			if (!(levelLine(cannyImg, Point(cols - 10, row - leftLen), leftEnd, 0, 40))) continue;
			Point rightEnd;
			if (!(levelLine(cannyImg, Point(cols + 10, row - rightLen), rightEnd, 1, 40))) continue;

			lastPoint.x = cols;
			lastPoint.y = row;
			cornerVec.push_back(Point(cols, row - 6));
			if (cornerVec.size() == 3) return true;
			else
			{
				for (int i = 0; i < 8; i++)
				{
					if ((int)cannyImg.at<uchar>(row - 10, cols + i) > 250)
					{
						row = row - 10;
						cols = cols + i;
						break;
					}
					if ((int)cannyImg.at<uchar>(row - 10, cols - i) > 250)
					{
						row = row - 10;
						cols = cols - i;
						break;
					}
				}
			}

		}
		else  continue; 
	}
	return false;
}

//����:�ǵ�; ���:�����Ľǵ�����
bool fixCornerfourDimension::rankCorner(vector<Point>& corner, int camerType)
{
	int size = corner.size();
	if (camerType == 0 || camerType == 3)
	{
		if (size != 27) return false;
	}
	else if (camerType == 1 || camerType == 2)
	{
		if (size != 18) return false;
	}

	vector<Point> temp(corner);
	if (camerType == 0)
	{
		for (int i = 0; i < 5; i++)
		{
			corner[i] = temp[14 - i];
		}
		for (int i = 0; i < 4; i++)
		{
			corner[5 + i] = temp[23 + i];
		}

		for (int i = 0; i < 5; i++)
		{
			corner[9+i] = temp[9 - i];
		}
		for (int i = 0; i < 4; i++)
		{
			corner[14 + i] = temp[19 + i];
		}

		for (int i = 0; i < 5; i++)
		{
			corner[18 + i] = temp[4 - i];
		}
		for (int i = 0; i < 4; i++)
		{
			corner[23 + i] = temp[15 + i];
		}
	}
	else if (camerType == 3)
	{
		for (int i = 0; i < 4; i++)
		{
			corner[i] = temp[18 - i];
		}
		for (int i = 0; i < 5; i++)
		{
			corner[4 + i] = temp[i];
		}

		for (int i = 0; i < 4; i++)
		{
			corner[9 + i] = temp[22 - i];
		}
		for (int i = 0; i < 5; i++)
		{
			corner[13 + i] = temp[5 + i];
		}

		for (int i = 0; i < 4; i++)
		{
			corner[18 + i] = temp[26 - i];
		}
		for (int i = 0; i < 5; i++)
		{
			corner[22 + i] = temp[10 + i];
		}
	}
	else if (camerType == 1)
	{
		for (int i = 0; i < 3; i++)
		{
			corner[i] = temp[17 - 2 * i];
		}
		for (int i = 0; i < 3; i++)
		{
			corner[3 + i] = temp[16 - 2 * i];
		}

		for (int i = 0; i < 4; i++)
		{
			corner[6 + 3 * i] = temp[8 + i];
		}

		for (int i = 0; i < 4; i++)
		{
			corner[7 + 3 * i] = temp[4 + i];
		}

		for (int i = 0; i < 4; i++)
		{
			corner[8 + 3 * i] = temp[0 + i];
		}
	}
	else if (camerType == 2)
	{
		for (int i = 0; i < 3; i++)
		{
			corner[i] = temp[2 + 3 * i];
		}
		for (int i = 0; i < 3; i++)
		{
			corner[3 + i] = temp[1 + 3 * i];
		}
		for (int i = 0; i < 3; i++)
		{
			corner[6 + i] = temp[0 + 3 * i];
		}

		for (int i = 0; i < 3; i++)
		{
			corner[9 + i] = temp[9 + 3 * i];
		}
		for (int i = 0; i < 3; i++)
		{
			corner[12 + i] = temp[10 + 3 * i];
		}
		for (int i = 0; i < 3; i++)
		{
			corner[15 + i] = temp[11 + 3 * i];
		}
	}
	return true;
}

bool fixCornerfourDimension::Line(Mat cannyImg, Point start, int minLen, int direction = 0)
{

	int row = start.y, cols = start.x;
	int maxRow = cannyImg.rows - 20, maxCols = cannyImg.cols - 20;
	int darkNum = 0;
	if (direction == 0)
	{
		while (1)
		{
			if (darkNum > 5 || row < 20 || row > maxRow || cols < 20 || cols > maxCols) return false;
			int move = 0, maxMove = 14;
			for (; move < maxMove; move++)
			{
				if ((int)cannyImg.at<uchar>(row - 1, cols + move) > 250) { row = row - 1; cols = cols + move; darkNum = 0; break; }
			}
			if (move == maxMove)
			{
				move = 0;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row - 1, cols - move) > 250) { row = row - 1; cols = cols - move; darkNum = 0; break; }
				}
				if (move == maxMove) { darkNum++; row--; }
			}
			int len = start.y - row;
			if (len > minLen) return true;
		}
	}
	else if (direction == 1)
	{
		int len = 0;
		while (1)
		{
			if (darkNum > 5 || row < 20 || row > maxRow || cols < 20 || cols > maxCols) return false;
			int move = 0, maxMove = 12;
			for (; move < maxMove; move++)
			{
				if ((int)cannyImg.at<uchar>(row + 1, cols - move) > 250) { row = row + 1; cols = cols - move; darkNum = 0; break; }
			}
			if (move == maxMove)
			{
				move = 0;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row + 1, cols + move) > 250) { row = row + 1; cols = cols + move; darkNum = 0; break; }
				}
				if (move == maxMove) { darkNum++; row++; }
			}
			len = row - start.y;
			if (len > minLen) return true;
		}
	}
	else return false;
}

bool fixCornerfourDimension::rightLine(Mat cannyImg, Point start, int minLen, int direction = 0)
{
	int row = start.y, cols = start.x;
	int maxRow = cannyImg.rows - 20, maxCols = cannyImg.cols - 20;
	int darkNum = 0;
	if (direction == 0)
	{
		int len = 0;
		while (1)
		{
			if (darkNum > 5 || row < 20 || row > maxRow || cols < 20 || cols > maxCols) return false;
			int move = 0, maxMove = 12;
			for (; move < maxMove; move++)
			{
				if ((int)cannyImg.at<uchar>(row - 1, cols - move) > 250) { row = row - 1; cols = cols - move; darkNum = 0; break; }
			}
			if (move == maxMove)
			{
				move = 0;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row - 1, cols + move) > 250) { row = row - 1; cols = cols + move; darkNum = 0; break; }
				}
				if (move == maxMove) { darkNum++; row--; }
			}
			len = start.y - row;
			if (len > minLen) return true;
		}
	}
	else if (direction == 1)
	{
		int len = 0;
		while (1)
		{
			if (darkNum > 5 || row < 20 || row > maxRow || cols < 20 || cols > maxCols) return false;
			int move = 0, maxMove = 9;
			for (; move < maxMove; move++)
			{
				if ((int)cannyImg.at<uchar>(row + 1, cols + move) > 250) { row = row + 1; cols = cols + move; darkNum = 0; break; }
			}
			if (move == maxMove)
			{
				move = 0;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row + 1, cols - move) > 250) { row = row + 1; cols = cols - move; darkNum = 0; break; }
				}
				if (move == maxMove) { darkNum++; row++; }
			}
			len = row - start.y;
			if (len > minLen) return true;
		}
	}

}

//����:ͼ������ͷ����type(0~3�ֱ�Ϊǰ���Һ�)�����:�ǵ���������corner(�����������)
bool fixCornerfourDimension::fixCorner(Mat src, int cameraType, vector<Point>& corner)
{
	
	Mat cannyImg = readProcess(src);	//Ԥ����ͼ��
	Point middleDownCorner(0,0);		//�������½ǵ������
	vector<Point> middleCornerVec;		//�����м������ǵ�
	if (!fixMiddleDownCorner(cannyImg, cameraType, middleDownCorner)) return false;
	if(!fixMiddleCorner(cannyImg, middleDownCorner, middleCornerVec)) return false;
	corner.clear();							//��սǵ�����
	int leftPointMinNum, rightPointMinNum;	//���ݾ�ͷ���ͣ��������ҽǵ�ĸ���
	if (cameraType == 0 || cameraType == 3){leftPointMinNum = 4;rightPointMinNum = 4;}
	else if (cameraType == 1){leftPointMinNum = 3;rightPointMinNum = 2;}
	else if (cameraType == 2){leftPointMinNum = 2;rightPointMinNum = 3;}
	else return false;

	//��ʼ�����ǵ�,���м��
	int xMaxEdge = cannyImg.cols - 20, yMaxEdge = cannyImg.rows - 20;		//���ñ߽�
	for (int line = 0; line < 3; line++)
	{
		int row = middleCornerVec[line].y, cols = middleCornerVec[line].x;	//��ʼ��̽�������
		int darkNum = 0, leftPointNum = 0;									//��ʼ���������������ǵ������
		Point lastPoint(middleCornerVec[line]);								//��ʼ����һ�ǵ������							
		int minDistance[4] = { 100,60,25,15 };								//����ͬ�нǵ����С���
		int distance = 0;													//��ʼ������һ�ǵ�ľ���
		corner.push_back(middleCornerVec[line]);
		for (int leftLen = 0; leftLen < 30; leftLen++)						//������,��ֹ̽�����ƫ
		{
			if ((int)cannyImg.at<uchar>(row + leftLen, cols - 30) > 250) { row = row + leftLen; cols = cols - 30; break; }
			if ((int)cannyImg.at<uchar>(row - leftLen, cols - 30) > 250) { row = row - leftLen; cols = cols - 30; break; }
		}

		while (leftPointNum < leftPointMinNum)	//�ҵ�����������ǵ�
		{
			//�����еĽǵ���
			if (line < 2)
			{
				//����̽���
				int move = 0, maxMove = 10;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row - move, cols - 1) > 250) { row = row - move; cols = cols - 1; break; }
					if ((int)cannyImg.at<uchar>(row - move, cols - 2) > 250) { row = row - move; cols = cols - 2; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols - 1) > 250) { row = row + move; cols = cols - 1; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols - 2) > 250) { row = row + move; cols = cols - 2; break; }
				}
				if (move == maxMove)
				{
					darkNum++;
					cols--;
					if (darkNum > 50)
					{
						if (leftPointNum == leftPointMinNum - 1)
						{
							corner.push_back(Point(lastPoint.x - 30, lastPoint.y - 10));
							leftPointNum++;
							break;
						}
					}
				}
				else darkNum = 0;

				//����̽��߽�
				if (row < 20 || row > yMaxEdge || cols < 20 || cols > xMaxEdge) break;

				//�жϾ���
				distance = lastPoint.x - cols;
				if (distance < minDistance[leftPointNum]) continue;

				//̽���Ϸ��Ƿ�������
				int top = 4, left = 0;
				for (; top < 7; top++)
				{
					if ((int)cannyImg.at<uchar>(row - top, cols) > 250) { break; }
					else if ((int)cannyImg.at<uchar>(row - top, cols - 1) > 250) { left = 1; break; }
					else if ((int)cannyImg.at<uchar>(row - top, cols + 1) > 250) { left = -1; break; }
				}
				if (top == 7) continue;

				//̽���Ϸ��Ƿ���б߳�
				if (!Line(cannyImg, Point(cols - left, row - top), 15, 0)) continue;

				//�����ǵ�����,��������
				cols = cols - top - left;
				corner.push_back(Point(cols, row));
				lastPoint = Point(cols, row);
				leftPointNum++;
				if (leftPointNum == leftPointMinNum) break;

				//�����ǰ�ǵ�
				for (int step = 0; step < 15; step++)
				{
					int topMove = 0, downMove = 0, maxMove = 12, topLeftMove = 1, downLeftMove = 1;
					for (; topMove < maxMove; topMove++)
					{
						if ((int)cannyImg.at<uchar>(row - topMove, cols - 1) > 250) { topLeftMove = 1; break; }
						if ((int)cannyImg.at<uchar>(row - topMove, cols - 2) > 250) { topLeftMove = 2; break; }
					}
					for (; downMove < maxMove; downMove++)
					{
						if ((int)cannyImg.at<uchar>(row + downMove, cols - 1) > 250) { downLeftMove = 1; break; }
						if ((int)cannyImg.at<uchar>(row + downMove, cols - 2) > 250) { downLeftMove = 2; break; }
					}
					if (topMove == maxMove && downMove == maxMove) cols--;
					else if (downMove == maxMove)
					{
						row = row - topMove;
						cols = cols - topLeftMove;
					}
					else if (topMove == maxMove)
					{
						row = row + downMove;
						cols = cols - downLeftMove;
					}
					else if (topMove - downMove < 6)
					{
						row = row - topMove;
						cols = cols - topLeftMove;
					}
					else if (topMove - downMove >= 6)
					{
						row = row + downMove;
						cols = cols - downLeftMove;
					}
					else cols--;
				}
			}
			//�����еĽǵ���
			else if (line == 2)
			{
				minDistance[3] = 30;
				int topMove = 0, downMove = 0, topLevelMove = 1, downLevelMove = 1, maxMove = 10;
				for (; topMove < maxMove; topMove++)
				{
					if ((int)cannyImg.at<uchar>(row - topMove, cols - 1) > 250) { topLevelMove = 1; break; }
					if ((int)cannyImg.at<uchar>(row - topMove, cols - 2) > 250) { topLevelMove = 2; break; }
				}

				for (; downMove < maxMove; downMove++)
				{
					if ((int)cannyImg.at<uchar>(row + downMove, cols - 1) > 250) { downLevelMove = 1; break; }
					if ((int)cannyImg.at<uchar>(row + downMove, cols - 2) > 250) { downLevelMove = 2; break; }
				}
				if (topMove == maxMove && downMove == maxMove)
				{
					darkNum++;
					cols--;
					if (darkNum > 50)
					{
						if (leftPointNum == leftPointMinNum - 1)
						{
							corner.push_back(Point(lastPoint.x - 30, lastPoint.y - 10));
							leftPointNum++;
							break;
						}
					}
				}
				else if (downMove == maxMove)
				{
					row = row - topMove;
					cols = cols - topLevelMove;
					darkNum = 0;
				}
				else if (topMove == maxMove)
				{
					row = row + downMove;
					cols = cols - downLevelMove;
					darkNum = 0;
				}
				else if (topMove < 5)
				{
					row = row - topMove;
					cols = cols - topLevelMove;
					darkNum = 0;
				}
				else if (topMove >= 5 && downMove < 5)
				{
					row = row + downMove;
					cols = cols - downLevelMove;
					darkNum = 0;
				}
				else cols--;

				//����̽��߽�
				if (row < 20 || row > yMaxEdge || cols < 20 || cols > xMaxEdge) break;

				//�жϾ���
				distance = lastPoint.x - cols;
				if (distance < minDistance[leftPointNum]) continue;

				//����·��Ƿ��нǵ�
				int move = 4, level = 0, maxM = 7;
				for (; move < maxM; move++)
				{
					if ((int)cannyImg.at<uchar>(row + move, cols) > 250) { level = 0; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols - 1) > 250) { level = 1; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols + 1) > 250) { level = -1; break; }
					
				}
				if (move == maxM) continue;
				
				//����·��Ƿ�����߶�
				if (!(Line(cannyImg, Point(cols - level, row + move), 15, 1))) continue;

				//����ǵ�����
				if (leftPointNum <= leftPointMinNum - 1)
				{
					corner.push_back(Point(cols - topMove, row));
				}
				else
				{
					corner.push_back(Point(cols, row));
				}
				
				lastPoint = Point(cols, row);
				leftPointNum++;
				if (leftPointNum == leftPointMinNum) break;
			}
		}
	}

	//��ʼ����ҽǵ�,���м��
	for (int line = 0; line < 3; line++)
	{
		int row = middleCornerVec[line].y, cols = middleCornerVec[line].x;	//��ʼ��̽�������
		int darkNum = 0, rightPointNum = 0;									//��ʼ���������������ǵ������
		Point lastPoint(middleCornerVec[line]);								//��ʼ����һ�ǵ������							
		int minDistance[4] = { 100,60,25,15 };								//����ͬ�нǵ����С���
		int distance = 0;													//��ʼ������һ�ǵ�ľ���

		//����ҵ�
		for (int i = 0; i < 60; i++)
		{
			if ((int)cannyImg.at<uchar>(row - i, cols + 30) > 250)
			{
				row = row - i;
				cols = cols + 30;
				break;
			}
			if ((int)cannyImg.at<uchar>(row - i, cols + 31) > 250)
			{
				row = row - i;
				cols = cols + 30;
				break;
			}
			if ((int)cannyImg.at<uchar>(row + i, cols + 30) > 250)
			{
				row = row + i;
				cols = cols + 30;
				break;
			}
			if ((int)cannyImg.at<uchar>(row + i, cols + 31) > 250)
			{
				row = row + i;
				cols = cols + 30;
				break;
			}
		}

		//���ҽǵ����������ʱ,�������
		while (rightPointNum < rightPointMinNum)
		{
			//�����еĽǵ���
			if (line < 2)
			{
				//����̽���
				int move = 0, maxMove = 10;
				for (; move < maxMove; move++)
				{
					if ((int)cannyImg.at<uchar>(row - move, cols + 1) > 250) { row = row - move; cols = cols + 1; break; }
					if ((int)cannyImg.at<uchar>(row - move, cols + 2) > 250) { row = row - move; cols = cols + 2; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols + 1) > 250) { row = row + move; cols = cols + 1; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols + 2) > 250) { row = row + move; cols = cols + 2; break; }
				}
				if (move == maxMove)
				{
					darkNum++;
					cols++;
					if (darkNum > 50)
					{
						if (rightPointNum == leftPointMinNum - 1)
						{
							corner.push_back(Point(lastPoint.x + 30, lastPoint.y - 10));
							rightPointNum++;
							break;
						}
					}
				}
				else darkNum = 0;

				//����̽��߽�
				if (row < 20 || row > yMaxEdge || cols < 20 || cols > xMaxEdge) break;

				//�жϾ���
				distance = cols - lastPoint.x;
				if (distance < minDistance[rightPointNum]) continue;

				//̽���Ϸ��Ƿ�������
				int top = 4, right = 0;
				for (; top < 7; top++)
				{
					if ((int)cannyImg.at<uchar>(row - top, cols) > 250) { break; }
					else if ((int)cannyImg.at<uchar>(row - top, cols + 1) > 250) { right = 1; break; }
					else if ((int)cannyImg.at<uchar>(row - top, cols - 1) > 250) { right = -1; break; }
				}
				if (top == 7) continue;

				//̽���Ϸ��Ƿ����߶�
				if (!(rightLine(cannyImg, Point(cols + right, row - top), 15, 0))) continue;

				//�����ǵ�����,��������
				cols = cols + top;
				row = row - right;
				corner.push_back(Point(cols, row));
				lastPoint = Point(cols, row);
				rightPointNum++;
				if (rightPointNum == rightPointMinNum) break;

				//�����ǰ�ǵ�
				for (int step = 0; step < 15; step++)
				{
					int topMove = 0, downMove = 0, rightTopMove = 1, rightDownMove =1, maxMove = 10;
					for (; topMove < maxMove; topMove++)
					{
						if ((int)cannyImg.at<uchar>(row - topMove, cols + 1) > 250) { rightTopMove = 1; break; }
						if ((int)cannyImg.at<uchar>(row - topMove, cols + 2) > 250) { rightTopMove = 2; break; }
					}
					for (; downMove < maxMove; downMove++)
					{
						if ((int)cannyImg.at<uchar>(row + downMove, cols + 1) > 250) { rightDownMove = 1; break; }
						if ((int)cannyImg.at<uchar>(row + downMove, cols + 2) > 250) { rightDownMove = 2; break; }
					}

					if (topMove == maxMove && downMove == maxMove) cols++;
					else if (downMove == maxMove)
					{
						row = row - topMove;
						cols = cols + rightTopMove;
					}
					else if (topMove == maxMove)
					{
						row = row + downMove;
						cols = cols + rightDownMove;
					}
					else if (topMove - downMove < 6)
					{
						row = row - topMove;
						cols = cols + rightTopMove;
					}
					else if (topMove - downMove >= 6)
					{
						row = row + downMove;
						cols = cols + rightDownMove;
					}
					else cols++;
				}
			}
			else if (line == 2)
			{
				minDistance[3] = 25;
				int topMove = 0, downMove = 0, topLevelMove = 1, downLevelMove = 1, maxMove = 10;
				for (; topMove < maxMove; topMove++)
				{
					if ((int)cannyImg.at<uchar>(row - topMove, cols + 1) > 250) { topLevelMove = 1; break; }
					if ((int)cannyImg.at<uchar>(row - topMove, cols + 2) > 250) { topLevelMove = 2; break; }
				}

				for (; downMove < maxMove; downMove++)
				{
					if ((int)cannyImg.at<uchar>(row + downMove, cols + 1) > 250) { downLevelMove = 1; break; }
					if ((int)cannyImg.at<uchar>(row + downMove, cols + 2) > 250) { downLevelMove = 2; break; }
				}
				if (topMove == maxMove && downMove == maxMove)
				{
					darkNum++;
					cols++;
					if (darkNum > 50)
					{
						if (rightPointNum == rightPointMinNum - 1)
						{
							corner.push_back(Point(lastPoint.x + 30, lastPoint.y - 10));
							rightPointNum++;
							break;
						}
					}
				}
				else if (downMove == maxMove)
				{
					row = row - topMove;
					cols = cols + topLevelMove;
					darkNum = 0;
				}
				else if (topMove == maxMove)
				{
					row = row + downMove;
					cols = cols + downLevelMove;
					darkNum = 0;
				}
				else if (topMove < 5)
				{
					row = row - topMove;
					cols = cols + topLevelMove;
					darkNum = 0;
				}
				else if (topMove >= 5 && downMove < 5)
				{
					row = row + downMove;
					cols = cols + downLevelMove;
					darkNum = 0;
				}
				else cols++;

				//����̽��߽�
				if (row < 20 || row > yMaxEdge || cols < 20 || cols > xMaxEdge) break;

				//�жϾ���
				distance = cols - lastPoint.x;
				if (distance < minDistance[rightPointNum]) continue;

				//����·��Ƿ��нǵ�
				int move = 4, level = 0, maxM = 7;
				for (; move < maxM; move++)
				{
					if ((int)cannyImg.at<uchar>(row + move, cols) > 250) { level = 0; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols + 1) > 250) { level = 1; break; }
					if ((int)cannyImg.at<uchar>(row + move, cols - 1) > 250) { level = -1; break; }

				}
				if (move == maxM) continue;

				//����·��Ƿ�����߶�
				if (!(rightLine(cannyImg, Point(cols + level, row + move), 15, 1))) continue;

				//����ǵ�����
				if (rightPointNum <= rightPointMinNum - 1)
				{
					corner.push_back(Point(cols + topMove, row));
				}
				else
				{
					corner.push_back(Point(cols, row));
				}

				lastPoint = Point(cols, row);
				rightPointNum++;
				if (rightPointNum == rightPointMinNum) break;
			}
		}
	}
	return true;
}
