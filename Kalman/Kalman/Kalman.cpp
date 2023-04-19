// Kalman.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <vector>
#include <math.h>
#include "IntputData.h"
#include "Eigen\Dense"

using namespace std;
using namespace Eigen;


int main()
{
	string filename = "GPS_result_static.txt";
	vector<vector<double>> X_real;
	vector<double> Time;
	double Long, Lat;
	Long = 0.00001141;
	Lat = 0.00000899;
	getInputData(filename.c_str(), X_real, Time);
	int row=X_real.size();
	int col = X_real[0].size();
	Matrix<double,3,3> Fai,P1,P2,K,Q,R,Tao;//P1:P(K,K-1)   P2:P(K-1)
	Fai << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	P2 << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	R << 25 * Lat * Lat, 0, 0,
		0, 25 * Long * Long, 0,
		0, 0, 25;
	Q << 0.001 * Lat * Lat, 0, 0,
		0, 0.001 * Long * Long, 0,
		0, 0, 0.001;
	Tao << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Matrix<double,3,3> H = Fai,I;
	I=MatrixXd::Identity(3, 3);
	Matrix<double,3,1> Z,X_1,X_2,P_diag;//X_1:X(K-1)  X_2:X(K,K-1)
	X_1 << 0, 0, 0;
	ofstream fout, fout2;
	fout.open("..\\..\\mat4.txt", ios::out | ios::binary);//在文件末尾追加写入
	fout.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
	fout.precision(11);  // 设置精度 
	for (int i = 0; i < row; i++)
	{
		Z << X_real[i][0], X_real[i][1], X_real[i][2];


		P_diag << sqrt(P2.data()[0]), sqrt(P2.data()[4]), sqrt(P2.data()[8]);
		P1 = Fai * P2 * Fai.transpose()+Tao*Q*Tao.transpose();
		K = P1 * H.transpose() * (H * P1 * H.transpose() + R).inverse();
		X_2 = Fai * X_1;
		X_1 = X_2 + K * (Z - H * X_2);
		P2 = (I - K * H) * P1 * (I - K * H).transpose() + K * R * K.transpose();
		fout << X_1.transpose() << " " << P_diag.transpose() << std::endl;//每次写完一个矩阵以后换行
	}

	fout.close();
	fout2.close();


   
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
