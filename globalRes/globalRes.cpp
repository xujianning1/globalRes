#include "stdafx.h"
//////////GPP/////////////////
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <memory>
//#include <opencv2/core/core.hpp>
//#include <opencv2/opencv.hpp>

#include "Gpp.h"
#include <Gpp.h>
//#include "RegistratePointCloud.h"
#include "TextureImage.h"
//#include "MyPointCloud.h"
#include "IntrinsicColor.h"
#include  "UnfoldMesh.h"
#include "DumpInfo.h"

//#include <Pcgl.h>
//#include <Parser.h>
//#include <ToolNNQuery.h>
#include<IPointCloud.h>
#include <PointCloud.h>
//#include "DataDefs.h"
using namespace std;
using namespace GPP;

bool readPoint(PointCloud *myPoiont, string &fileName) {
	ifstream input;
	input.open(fileName);
	if (!input.is_open()) {
		return false;
	}
	while (!input.eof()) {
		double x = 0.0, y = 0.0, z = 0.0, nx = 0.0, ny = 0.0, nz = 0.0;
		input >> x >> y >> z >> nx >> ny >> nz;
		Vector3 coord(x, y, z);
		Vector3 norm(nx, ny, nz);
		myPoiont->InsertPoint(coord, norm);
		myPoiont->SetHasNormal(true);
	}

	return true;
}

bool readImageidList(std::vector< std::vector< GPP::ImageColorId > >& imageColorIdList, const vector<string> &fileList) {
	int index = 0;
	for (int i = 0; i < fileList.size(); i++)
	{
		ifstream imageMap(fileList[i]);
		if (!imageMap.is_open()) {
			cout << "无法打开文件" << endl;
			return false;
		}
		std::vector< GPP::ImageColorId > imageId;
		
		while (!imageMap.eof()) {
			int imageX = 0, imageY = 0;
			imageMap >> imageX >> imageY;
			imageId.emplace_back(ImageColorId(index++, imageX, imageY));
		}
		imageColorIdList.emplace_back(imageId);
	}
	return true;
}

bool Registration(const vector<IPointCloud*> pointCloudList, Int MaxIterationCount,
	std::vector< GPP::Matrix4x4 > &resultTransform, const string &fileName, const vector<double>& matrixVal) {
	std::vector<GPP::Matrix4x4> initTransformList;

	for (int i = 0; i < pointCloudList.size(); i++)
	{
		GPP::Matrix4x4 MatrixTemp(0);
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				MatrixTemp.SetValue(j, k, matrixVal[j * 4 + k]);
			}
		}
		initTransformList.emplace_back(MatrixTemp);
	}
	GPP::DumpOnce();
	//GPP::ErrorCode res = GPP::RegistratePointCloud::GlobalRegistrate(&pointCloudList, 10, &resultTransform, NULL, true, 0, NULL);
	
	std::vector<Vector3> marksRef;
	std::vector<Vector3> marksFrom;
	Matrix4x4 resultTransform_temp;
	//GPP::ErrorCode res = GPP::RegistratePointCloud::ICPRegistrate(pointCloudList[0], &marksRef, pointCloudList[1], &marksFrom, &resultTransform_temp);
	GPP::ErrorCode res = GPP::RegistratePointCloud::AlignPointCloud(pointCloudList[0],pointCloudList[1], &resultTransform_temp);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << resultTransform_temp.GetValue(i, j) << "\t";
		}
		cout << endl;
	}
	if (res != GPP_NO_ERROR)
	{
		cout << "全局注册失败" << endl;
		return false;
	}
	int pointListCount = pointCloudList.size();
	ofstream matrixFile("matrix.txt");
	for (int cloudid = 0; cloudid < pointListCount; cloudid++)
	{
		//char path[1024];
		//sprintf(path, "result_%d.asc", cloudid + 1);
		string path = fileName + to_string(static_cast<long long> (cloudid)) + ".asc";
		ofstream out(path);
		GPP::IPointCloud* curPointCloud = pointCloudList.at(cloudid);
		int curPointCount = curPointCloud->GetPointCount();
		for (int pid = 0; pid < curPointCount; pid++)
		{
			curPointCloud->SetPointCoord(pid, resultTransform.at(cloudid).TransformPoint(curPointCloud->GetPointCoord(pid)));
			curPointCloud->SetPointNormal(pid, resultTransform.at(cloudid).RotateVector(curPointCloud->GetPointNormal(pid)));
			out << curPointCloud->GetPointCoord(pid)[0] << "\t" << curPointCloud->GetPointCoord(pid)[1] << "\t" << curPointCloud->GetPointCoord(pid)[2] << "\t"
				<< curPointCloud->GetPointNormal(pid)[0] << "\t" << curPointCloud->GetPointNormal(pid)[1] << "\t" << curPointCloud->GetPointNormal(pid)[2] << endl;
		}
		out.close();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				matrixFile << resultTransform.at(cloudid).GetValue(i, j) << "\t";
			}
			matrixFile << endl;
		}
		matrixFile << endl;
	}
	matrixFile.close();
	return true;
};

int main(int argc, char* argv[]) {
	bool result = true;
	result = GPP::SetActivationKey("f711c835994e97d1173693266217bd604909492b81d8036649");   //最老GPP
	//result = GPP::SetPcglActivationKey("4ba05e65da9cc6dc9b9438b188369922");  //20181114鏂扮増PCGL
	//"10P.asc" "10P_0_5_10_30_30_30_Rao.asc" "10P_10_10_10_45_45_45_Rao.asc" "10P_-10_-10_-10_-45_-45_-45_Rao.asc"
	if (result == false)
	{
		cout << "失败" << endl;
	}
	cout << "成功激活" << endl;

	std::vector< GPP::Matrix4x4 > resultTransform;
	//vector<string> fileNameList;
	//string filename1("DesignModel.asc");
	//string filename2("ScanModel.asc");
	////string filename3("yaokongqi_3.asc");
	//fileNameList.emplace_back(filename1);
	//fileNameList.emplace_back(filename2);
	//fileNameList.emplace_back(filename3);
	vector<IPointCloud*>  PointsList;
	for (int i = 1; i < argc; i++)
	{
		PointCloud *myPoint = new PointCloud();
		//if (!readPoint(myPoint, fileNameList[i])) {
		string fileTemp(argv[i]);
		if (!readPoint(myPoint, fileTemp)) {
			cout << "read file fail" << endl;
		}
		PointsList.emplace_back(myPoint);
	}

	//-8.151610, -40.192798, 404.221982 
	//- 6.254580, 3.638400, 390.417993
	//- 16.461100, 32.033797, 384.519011
	//- 1.115600, 28.784700, 378.877997
	//- 13.302300, 63.317999, 371.640980

	//-4.617558, -40.750312, 404.221982 
	//-6.547887, 3.079432, 390.417993 
	//-19.190390, 30.477220, 384.519011 
	//-3.620107, 28.577934, 378.877997 
	//-18.770208, 61.917683, 371.640980 
	std::vector<Vector3> marksRef;
	marksRef.emplace_back(Vector3(-8.151610, -40.192798, 404.221982));
	marksRef.emplace_back(Vector3(-6.254580, 3.638400, 390.417993));
	marksRef.emplace_back(Vector3(-16.461100, 32.033797, 384.519011));
	marksRef.emplace_back(Vector3(-1.115600, 28.784700, 378.877997));
	marksRef.emplace_back(Vector3(-13.302300, 63.317999, 371.640980));
	std::vector<Vector3> marksFrom;
	marksFrom.emplace_back(Vector3(-4.617558, -40.750312, 404.221982));
	marksFrom.emplace_back(Vector3(-6.547887, 3.079432, 390.417993));
	marksFrom.emplace_back(Vector3(-19.190390, 30.477220, 384.519011));
	marksFrom.emplace_back(Vector3(-3.620107, 28.577934, 378.877997));
	marksFrom.emplace_back(Vector3(-18.770208, 61.917683, 371.640980));

	for (int i = 1; i < PointsList.size(); i++)
	{
		Matrix4x4 resultTransformTemp;
		//GPP::ErrorCode res = RegistratePointCloud::AlignPointCloud(PointsList[i - 1], PointsList[i], &resultTransformTemp);
		GPP::ErrorCode res = RegistratePointCloud::AlignPointCloudByMark(PointsList[i - 1], &marksRef, PointsList[i], &marksFrom, &resultTransformTemp);
		if (res != GPP_NO_ERROR)
		{
			cout << "第" << i << "幅点云粗配准失败" << endl;
			return 0;
		}
		resultTransform.emplace_back(resultTransformTemp);
		cout << resultTransformTemp.GetValue(0) << "\t"  << resultTransformTemp.GetValue(1) << "\t" << resultTransformTemp.GetValue(2) << "\t" << resultTransformTemp.GetValue(3) << endl
		<< resultTransformTemp.GetValue(4) << "\t" << resultTransformTemp.GetValue(5) << "\t" << resultTransformTemp.GetValue(6) << "\t" << resultTransformTemp.GetValue(7) << endl
		<< resultTransformTemp.GetValue(8) << "\t" << resultTransformTemp.GetValue(9) << "\t" << resultTransformTemp.GetValue(10) << "\t" << resultTransformTemp.GetValue(11) << endl
		<< resultTransformTemp.GetValue(12) << "\t" << resultTransformTemp.GetValue(13) << "\t" << resultTransformTemp.GetValue(14) << "\t" << resultTransformTemp.GetValue(15) << endl;
		//ofstream out1("zhucewancheng2.asc");
		int curPointCount = PointsList[i]->GetPointCount();
		int distanceOfAllPoints = 0;
		for (int pid = 0; pid < curPointCount; pid++)
		{
			PointsList[i]->SetPointCoord(pid, resultTransform.back().TransformPoint(PointsList[i]->GetPointCoord(pid)));
			PointsList[i]->SetPointNormal(pid, resultTransform.back().RotateVector(PointsList[i]->GetPointNormal(pid)));

			/*out1 << PointsList[i]->GetPointCoord(pid)[0] << "\t" << PointsList[i]->GetPointCoord(pid)[1] << "\t" << PointsList[i]->GetPointCoord(pid)[2] << "\t"
				<< PointsList[i]->GetPointNormal(pid)[0] << "\t" << PointsList[i]->GetPointNormal(pid)[1] << "\t" << PointsList[i]->GetPointNormal(pid)[2] << endl;*/
		}
		//out1.close();
	}

	ifstream inMatrix("P10M.txt");
	vector<double> matrixVal;
	if (!inMatrix) {
		cout << "file fail" << endl;
	}
	while (!inMatrix.eof()) {
		double M1 = 0.0, M2 = 0.0, M3 = 0.0, M4 = 0.0;
		inMatrix >> M1 >> M2 >> M3 >> M4;
		matrixVal.emplace_back(M1);
		matrixVal.emplace_back(M2);
		matrixVal.emplace_back(M3);
		matrixVal.emplace_back(M4);
	}
	inMatrix.close();

	std::vector< GPP::Matrix4x4 > resultTransformOfglob;
	string fileResultName(argv[1]);
	string fileResult(fileResultName.begin(), fileResultName.end() - 4);
	fileResult += "_result.asc";
	bool res = Registration(PointsList, 10, resultTransformOfglob, fileResult, matrixVal);
	if (!res)
	{
		cout << "全局注册失败" << endl;
		return 0;
	}
	return 0;
}