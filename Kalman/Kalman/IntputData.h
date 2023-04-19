#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;


const double  axis1 = 6378137;
const double  axis2 = 6356752;
const double pi = acos(-1);
double e1, e2;
bool getInputData(const char* fileName, vector<vector<double>>& Mat, vector<double>& Time)
{
	ifstream fileStream;
	string oneLine = "";	//输入文件的某一行
	double tmp = 0;		//当前位置上的数值
	int rowCount;	// 行数计数器
	int colCount = 0;	// 列数计数器
	int index = 0;		// 当前位置在X[]数组的下标

	// 打开文件
	fileStream.open(fileName, ios::in);	//ios::in 表示以只读的方式读取文件
	if (fileStream.fail())//文件打开失败:返回0
	{
		cout << "文件不存在." << endl;
		fileStream.close();
		system("pause");
		return 0;
	}
	else//文件存在
	{
		// 读入数据
		rowCount = 0;	//初始化当前行数为0
		colCount = 0;	//当前列数清零
		double tmp = 0;	//当前读入的数值
		vector<double> row;
		while (!fileStream.eof()) {
			fileStream >> tmp;
			if (colCount == 0)
			{
				Time.push_back(tmp);
			}
			else
			{
				row.push_back(tmp);
			}

			if ('\n' != fileStream.peek()&& !fileStream.eof()) {	// 未到行尾，colCount累加，rowCount不变
				++colCount;
			}
			else {	//已到行尾，colCount清零，rowCount累加				
				rowCount++;	// 换下一行
				colCount = 0;	// 列数清零	
				Mat.push_back(row);
				row.clear();
			}
		}
		// 关闭文件
		fileStream.close();
		return 1;
	}
}// END OF getInputData

void transform_axis(vector<vector<double>>& Mat, int row, int col)
{
	double B,L,N, H;
	e1 = (axis1 * axis1 - axis2 * axis2) / (axis1 * axis1);
	for (int i = 0; i < row; i++)
	{
		B = Mat[i][0] * pi / 180;
		L = Mat[i][1] * pi / 180;
		H = Mat[i][2];
		N = axis1 / sqrt(1 - e1 * pow(sin(B), 2));
		Mat[i][0] = (N + H) * cos(B) * cos(L);
		Mat[i][1] = (N + H) * cos(B) * sin(L);
		Mat[i][2] = (N * (1 - e1) + H) * sin(B);
	}
}

Vector3<double> axis_transform(const Vector3<double>& vec)
{
	Vector3<double> vec1;
	double x, y, z, P, theta;
	x = vec[0];
	y = vec[1];
	z = vec[2];
	P = sqrt(x * x + y * y);
	e2 = (axis1 * axis1 - axis2 * axis2) / (axis2 * axis2);
	theta = atan(z * axis1 / (P * axis2));
	vec1[0] = atan((z + e2 * axis2 * pow(sin(theta), 3)) / (P - e1 * axis1 * (pow(cos(theta), 3)))) * 180 / pi;
	vec1[1] = atan(y / x) * 180 / pi + 180;
	vec1[2] = P / (cos(vec[0]));
	return vec1;
}
