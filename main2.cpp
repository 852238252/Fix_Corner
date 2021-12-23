#include"Four_Dimension5.h"


int main()
{
	//计时开始
	clock_t startTime, endTime;
	startTime = clock();

	//检测四幅图像的角点
	for (int i = 1; i < 5; i++)
	{
		string imgPath = "F:\\Data\\test5\\4\\";					//图像路径
		string imgName = to_string(i);								//图像名称
		Mat src = imread(imgPath + imgName + ".jpg", 0);			//读取图片
		fixCornerfourDimension f;									//实例化四维布角点检测类
		Mat grayImg = f.readProcess(src);
		imwrite(imgPath + imgName + "_canny.jpg", grayImg);
		vector<Point> corner;										//定义角点存储容器
		
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

	//计时结束，显示时间
	endTime = clock();
	cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	waitKey(0);
	return 0;
}