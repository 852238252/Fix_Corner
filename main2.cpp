#include"Four_Dimension5.h"


int main()
{
	//��ʱ��ʼ
	clock_t startTime, endTime;
	startTime = clock();

	//����ķ�ͼ��Ľǵ�
	for (int i = 1; i < 5; i++)
	{
		string imgPath = "F:\\Data\\test5\\4\\";					//ͼ��·��
		string imgName = to_string(i);								//ͼ������
		Mat src = imread(imgPath + imgName + ".jpg", 0);			//��ȡͼƬ
		fixCornerfourDimension f;									//ʵ������ά���ǵ�����
		Mat grayImg = f.readProcess(src);
		imwrite(imgPath + imgName + "_canny.jpg", grayImg);
		vector<Point> corner;										//����ǵ�洢����
		
		if (f.fixCorner(src, i - 1, corner))
		{
			Mat colorImg = imread(imgPath + imgName + ".jpg", 1);
			for (int i = 0; i < corner.size(); i++)
			{
				circle(colorImg, corner[i], 6, Scalar(0, 255, 0), 2);
				putText(colorImg, to_string(i), corner[i], FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
			}
			imshow(to_string(i), colorImg);
		}
		else{cout << "cam  " << i << "can not find the corners!!!" << endl;}
	}

	//��ʱ��������ʾʱ��
	endTime = clock();
	cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	waitKey(0);
	return 0;
}