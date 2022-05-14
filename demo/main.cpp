
#include <iostream>
#include <fstream>
#include <mutex>
#include <windows.h>
#include <stdio.h>
#include <thread>
#include <fstream>
#include "direct.h"
#include "Logger.h"
#include "Capture.h"
#include "common.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

static Capture capture;
static bool s_bSaveFrameFlg = false;
std::vector<int> deviceIDs;
static bool* s_CamPicFlag;
static std::string folderName;
#ifdef CFG_OPENCV_ON
void depth2color(cv::Mat& color, const cv::Mat& depth, const double max, const double min)
{
	cv::Mat grayImage;
	double alpha = 255.0 / (max - min);
	depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min);
	cv::applyColorMap(grayImage, color, cv::COLORMAP_JET);
}
#endif

#define MAX_DEPTH (3000.0)
float turbo_srgb_floats[256][3] = { {0.18995,0.07176,0.23217},{0.19483,0.08339,0.26149},{0.19956,0.09498,0.29024},{0.20415,0.10652,0.31844},{0.20860,0.11802,0.34607},{0.21291,0.12947,0.37314},{0.21708,0.14087,0.39964},{0.22111,0.15223,0.42558},{0.22500,0.16354,0.45096},{0.22875,0.17481,0.47578},{0.23236,0.18603,0.50004},{0.23582,0.19720,0.52373},{0.23915,0.20833,0.54686},{0.24234,0.21941,0.56942},{0.24539,0.23044,0.59142},{0.24830,0.24143,0.61286},{0.25107,0.25237,0.63374},{0.25369,0.26327,0.65406},{0.25618,0.27412,0.67381},{0.25853,0.28492,0.69300},{0.26074,0.29568,0.71162},{0.26280,0.30639,0.72968},{0.26473,0.31706,0.74718},{0.26652,0.32768,0.76412},{0.26816,0.33825,0.78050},{0.26967,0.34878,0.79631},{0.27103,0.35926,0.81156},{0.27226,0.36970,0.82624},{0.27334,0.38008,0.84037},{0.27429,0.39043,0.85393},{0.27509,0.40072,0.86692},{0.27576,0.41097,0.87936},{0.27628,0.42118,0.89123},{0.27667,0.43134,0.90254},{0.27691,0.44145,0.91328},{0.27701,0.45152,0.92347},{0.27698,0.46153,0.93309},{0.27680,0.47151,0.94214},{0.27648,0.48144,0.95064},{0.27603,0.49132,0.95857},{0.27543,0.50115,0.96594},{0.27469,0.51094,0.97275},{0.27381,0.52069,0.97899},{0.27273,0.53040,0.98461},{0.27106,0.54015,0.98930},{0.26878,0.54995,0.99303},{0.26592,0.55979,0.99583},{0.26252,0.56967,0.99773},{0.25862,0.57958,0.99876},{0.25425,0.58950,0.99896},{0.24946,0.59943,0.99835},{0.24427,0.60937,0.99697},{0.23874,0.61931,0.99485},{0.23288,0.62923,0.99202},{0.22676,0.63913,0.98851},{0.22039,0.64901,0.98436},{0.21382,0.65886,0.97959},{0.20708,0.66866,0.97423},{0.20021,0.67842,0.96833},{0.19326,0.68812,0.96190},{0.18625,0.69775,0.95498},{0.17923,0.70732,0.94761},{0.17223,0.71680,0.93981},{0.16529,0.72620,0.93161},{0.15844,0.73551,0.92305},{0.15173,0.74472,0.91416},{0.14519,0.75381,0.90496},{0.13886,0.76279,0.89550},{0.13278,0.77165,0.88580},{0.12698,0.78037,0.87590},{0.12151,0.78896,0.86581},{0.11639,0.79740,0.85559},{0.11167,0.80569,0.84525},{0.10738,0.81381,0.83484},{0.10357,0.82177,0.82437},{0.10026,0.82955,0.81389},{0.09750,0.83714,0.80342},{0.09532,0.84455,0.79299},{0.09377,0.85175,0.78264},{0.09287,0.85875,0.77240},{0.09267,0.86554,0.76230},{0.09320,0.87211,0.75237},{0.09451,0.87844,0.74265},{0.09662,0.88454,0.73316},{0.09958,0.89040,0.72393},{0.10342,0.89600,0.71500},{0.10815,0.90142,0.70599},{0.11374,0.90673,0.69651},{0.12014,0.91193,0.68660},{0.12733,0.91701,0.67627},{0.13526,0.92197,0.66556},{0.14391,0.92680,0.65448},{0.15323,0.93151,0.64308},{0.16319,0.93609,0.63137},{0.17377,0.94053,0.61938},{0.18491,0.94484,0.60713},{0.19659,0.94901,0.59466},{0.20877,0.95304,0.58199},{0.22142,0.95692,0.56914},{0.23449,0.96065,0.55614},{0.24797,0.96423,0.54303},{0.26180,0.96765,0.52981},{0.27597,0.97092,0.51653},{0.29042,0.97403,0.50321},{0.30513,0.97697,0.48987},{0.32006,0.97974,0.47654},{0.33517,0.98234,0.46325},{0.35043,0.98477,0.45002},{0.36581,0.98702,0.43688},{0.38127,0.98909,0.42386},{0.39678,0.99098,0.41098},{0.41229,0.99268,0.39826},{0.42778,0.99419,0.38575},{0.44321,0.99551,0.37345},{0.45854,0.99663,0.36140},{0.47375,0.99755,0.34963},{0.48879,0.99828,0.33816},{0.50362,0.99879,0.32701},{0.51822,0.99910,0.31622},{0.53255,0.99919,0.30581},{0.54658,0.99907,0.29581},{0.56026,0.99873,0.28623},{0.57357,0.99817,0.27712},{0.58646,0.99739,0.26849},{0.59891,0.99638,0.26038},{0.61088,0.99514,0.25280},{0.62233,0.99366,0.24579},{0.63323,0.99195,0.23937},{0.64362,0.98999,0.23356},{0.65394,0.98775,0.22835},{0.66428,0.98524,0.22370},{0.67462,0.98246,0.21960},{0.68494,0.97941,0.21602},{0.69525,0.97610,0.21294},{0.70553,0.97255,0.21032},{0.71577,0.96875,0.20815},{0.72596,0.96470,0.20640},{0.73610,0.96043,0.20504},{0.74617,0.95593,0.20406},{0.75617,0.95121,0.20343},{0.76608,0.94627,0.20311},{0.77591,0.94113,0.20310},{0.78563,0.93579,0.20336},{0.79524,0.93025,0.20386},{0.80473,0.92452,0.20459},{0.81410,0.91861,0.20552},{0.82333,0.91253,0.20663},{0.83241,0.90627,0.20788},{0.84133,0.89986,0.20926},{0.85010,0.89328,0.21074},{0.85868,0.88655,0.21230},{0.86709,0.87968,0.21391},{0.87530,0.87267,0.21555},{0.88331,0.86553,0.21719},{0.89112,0.85826,0.21880},{0.89870,0.85087,0.22038},{0.90605,0.84337,0.22188},{0.91317,0.83576,0.22328},{0.92004,0.82806,0.22456},{0.92666,0.82025,0.22570},{0.93301,0.81236,0.22667},{0.93909,0.80439,0.22744},{0.94489,0.79634,0.22800},{0.95039,0.78823,0.22831},{0.95560,0.78005,0.22836},{0.96049,0.77181,0.22811},{0.96507,0.76352,0.22754},{0.96931,0.75519,0.22663},{0.97323,0.74682,0.22536},{0.97679,0.73842,0.22369},{0.98000,0.73000,0.22161},{0.98289,0.72140,0.21918},{0.98549,0.71250,0.21650},{0.98781,0.70330,0.21358},{0.98986,0.69382,0.21043},{0.99163,0.68408,0.20706},{0.99314,0.67408,0.20348},{0.99438,0.66386,0.19971},{0.99535,0.65341,0.19577},{0.99607,0.64277,0.19165},{0.99654,0.63193,0.18738},{0.99675,0.62093,0.18297},{0.99672,0.60977,0.17842},{0.99644,0.59846,0.17376},{0.99593,0.58703,0.16899},{0.99517,0.57549,0.16412},{0.99419,0.56386,0.15918},{0.99297,0.55214,0.15417},{0.99153,0.54036,0.14910},{0.98987,0.52854,0.14398},{0.98799,0.51667,0.13883},{0.98590,0.50479,0.13367},{0.98360,0.49291,0.12849},{0.98108,0.48104,0.12332},{0.97837,0.46920,0.11817},{0.97545,0.45740,0.11305},{0.97234,0.44565,0.10797},{0.96904,0.43399,0.10294},{0.96555,0.42241,0.09798},{0.96187,0.41093,0.09310},{0.95801,0.39958,0.08831},{0.95398,0.38836,0.08362},{0.94977,0.37729,0.07905},{0.94538,0.36638,0.07461},{0.94084,0.35566,0.07031},{0.93612,0.34513,0.06616},{0.93125,0.33482,0.06218},{0.92623,0.32473,0.05837},{0.92105,0.31489,0.05475},{0.91572,0.30530,0.05134},{0.91024,0.29599,0.04814},{0.90463,0.28696,0.04516},{0.89888,0.27824,0.04243},{0.89298,0.26981,0.03993},{0.88691,0.26152,0.03753},{0.88066,0.25334,0.03521},{0.87422,0.24526,0.03297},{0.86760,0.23730,0.03082},{0.86079,0.22945,0.02875},{0.85380,0.22170,0.02677},{0.84662,0.21407,0.02487},{0.83926,0.20654,0.02305},{0.83172,0.19912,0.02131},{0.82399,0.19182,0.01966},{0.81608,0.18462,0.01809},{0.80799,0.17753,0.01660},{0.79971,0.17055,0.01520},{0.79125,0.16368,0.01387},{0.78260,0.15693,0.01264},{0.77377,0.15028,0.01148},{0.76476,0.14374,0.01041},{0.75556,0.13731,0.00942},{0.74617,0.13098,0.00851},{0.73661,0.12477,0.00769},{0.72686,0.11867,0.00695},{0.71692,0.11268,0.00629},{0.70680,0.10680,0.00571},{0.69650,0.10102,0.00522},{0.68602,0.09536,0.00481},{0.67535,0.08980,0.00449},{0.66449,0.08436,0.00424},{0.65345,0.07902,0.00408},{0.64223,0.07380,0.00401},{0.63082,0.06868,0.00401},{0.61923,0.06367,0.00410},{0.60746,0.05878,0.00427},{0.59550,0.05399,0.00453},{0.58336,0.04931,0.00486},{0.57103,0.04474,0.00529},{0.55852,0.04028,0.00579},{0.54583,0.03593,0.00638},{0.53295,0.03169,0.00705},{0.51989,0.02756,0.00780},{0.50664,0.02354,0.00863},{0.49321,0.01963,0.00955},{0.47960,0.01583,0.01055} };
unsigned char turbo_srgb_bytes[256][3] = { {48,18,59},{50,21,67},{51,24,74},{52,27,81},{53,30,88},{54,33,95},{55,36,102},{56,39,109},{57,42,115},{58,45,121},{59,47,128},{60,50,134},{61,53,139},{62,56,145},{63,59,151},{63,62,156},{64,64,162},{65,67,167},{65,70,172},{66,73,177},{66,75,181},{67,78,186},{68,81,191},{68,84,195},{68,86,199},{69,89,203},{69,92,207},{69,94,211},{70,97,214},{70,100,218},{70,102,221},{70,105,224},{70,107,227},{71,110,230},{71,113,233},{71,115,235},{71,118,238},{71,120,240},{71,123,242},{70,125,244},{70,128,246},{70,130,248},{70,133,250},{70,135,251},{69,138,252},{69,140,253},{68,143,254},{67,145,254},{66,148,255},{65,150,255},{64,153,255},{62,155,254},{61,158,254},{59,160,253},{58,163,252},{56,165,251},{55,168,250},{53,171,248},{51,173,247},{49,175,245},{47,178,244},{46,180,242},{44,183,240},{42,185,238},{40,188,235},{39,190,233},{37,192,231},{35,195,228},{34,197,226},{32,199,223},{31,201,221},{30,203,218},{28,205,216},{27,208,213},{26,210,210},{26,212,208},{25,213,205},{24,215,202},{24,217,200},{24,219,197},{24,221,194},{24,222,192},{24,224,189},{25,226,187},{25,227,185},{26,228,182},{28,230,180},{29,231,178},{31,233,175},{32,234,172},{34,235,170},{37,236,167},{39,238,164},{42,239,161},{44,240,158},{47,241,155},{50,242,152},{53,243,148},{56,244,145},{60,245,142},{63,246,138},{67,247,135},{70,248,132},{74,248,128},{78,249,125},{82,250,122},{85,250,118},{89,251,115},{93,252,111},{97,252,108},{101,253,105},{105,253,102},{109,254,98},{113,254,95},{117,254,92},{121,254,89},{125,255,86},{128,255,83},{132,255,81},{136,255,78},{139,255,75},{143,255,73},{146,255,71},{150,254,68},{153,254,66},{156,254,64},{159,253,63},{161,253,61},{164,252,60},{167,252,58},{169,251,57},{172,251,56},{175,250,55},{177,249,54},{180,248,54},{183,247,53},{185,246,53},{188,245,52},{190,244,52},{193,243,52},{195,241,52},{198,240,52},{200,239,52},{203,237,52},{205,236,52},{208,234,52},{210,233,53},{212,231,53},{215,229,53},{217,228,54},{219,226,54},{221,224,55},{223,223,55},{225,221,55},{227,219,56},{229,217,56},{231,215,57},{233,213,57},{235,211,57},{236,209,58},{238,207,58},{239,205,58},{241,203,58},{242,201,58},{244,199,58},{245,197,58},{246,195,58},{247,193,58},{248,190,57},{249,188,57},{250,186,57},{251,184,56},{251,182,55},{252,179,54},{252,177,54},{253,174,53},{253,172,52},{254,169,51},{254,167,50},{254,164,49},{254,161,48},{254,158,47},{254,155,45},{254,153,44},{254,150,43},{254,147,42},{254,144,41},{253,141,39},{253,138,38},{252,135,37},{252,132,35},{251,129,34},{251,126,33},{250,123,31},{249,120,30},{249,117,29},{248,114,28},{247,111,26},{246,108,25},{245,105,24},{244,102,23},{243,99,21},{242,96,20},{241,93,19},{240,91,18},{239,88,17},{237,85,16},{236,83,15},{235,80,14},{234,78,13},{232,75,12},{231,73,12},{229,71,11},{228,69,10},{226,67,10},{225,65,9},{223,63,8},{221,61,8},{220,59,7},{218,57,7},{216,55,6},{214,53,6},{212,51,5},{210,49,5},{208,47,5},{206,45,4},{204,43,4},{202,42,4},{200,40,3},{197,38,3},{195,37,3},{193,35,2},{190,33,2},{188,32,2},{185,30,2},{183,29,2},{180,27,1},{178,26,1},{175,24,1},{172,23,1},{169,22,1},{167,20,1},{164,19,1},{161,18,1},{158,16,1},{155,15,1},{152,14,1},{149,13,1},{146,11,1},{142,10,1},{139,9,2},{136,8,2},{133,7,2},{129,6,2},{126,5,2},{122,4,3} };


cv::Vec3b interpolate(float colormap[][3], float x) {
	x = MAX(0.0, MIN(1.0, x));
	int a = int(x * 255.0);
	int b = MIN(255, a + 1);
	float f = x * 255.0 - a;
	cv::Vec3b ret;
	ret[2] = floor((colormap[a][0] + (colormap[b][0] - colormap[a][0]) * f) * 255);
	ret[1] = floor((colormap[a][1] + (colormap[b][1] - colormap[a][1]) * f) * 255);
	ret[0] = floor((colormap[a][2] + (colormap[b][2] - colormap[a][2]) * f) * 255);
	return ret;
}

//  主函数
void Depth2ColorMapTurbo(const cv::Mat& depth_image, cv::Mat& depth_color_image)
{
	int rows = depth_image.rows;
	int cols = depth_image.cols;
	cv::Mat floatMat;
	depth_image.convertTo(floatMat, CV_32FC1);
	for (int y = 0; y < rows; y++) {
		cv::Vec3b* data = depth_color_image.ptr<cv::Vec3b>(y);
		for (int x = 0; x < cols; x++) {
			float d = floatMat.at<float>(y, x);
			if (d == 0.0 || d >= MAX_DEPTH) {
				data[x][0] = data[x][1] = data[x][2] = 0.0;
			}
			else {
				d = fabs(d - MAX_DEPTH) / MAX_DEPTH;
				data[x] = interpolate(turbo_srgb_floats, d);
				//data[x] = TurboColormap(d);
			}
		}
	}
	return;
}

static long long GetSysCurrentTime()
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	long long curTime = ((long long)(sys.wSecond)) * 1000 + sys.wMilliseconds / 1000;
	return curTime;
}

static double getFrameRate()
{
	static double fps = 0.0;
	static int frameCount = 0;
	static auto lastTime = GetCurrentTime();
	static auto curTime = GetCurrentTime();

	curTime = GetCurrentTime();
	auto duration = (curTime - lastTime) / 1000;

	if (duration > 2) {
		fps = frameCount / duration;
		frameCount = 0;
		lastTime = curTime;
	}

	++frameCount;
	return fps;
}


static void AS_SDK_DepthImageCallback(int deviceId, const AS_DEPTHIMAGE_Data* pstFrame, void* privateData)
{
	static bool bFirst = true;
	static bool bFirstCam = false;
	static bool bSecondCam = false;
	double frameRate = getFrameRate();
	if (frameRate != 0 && bFirst) {
		LOG(INFO) << "FrameRate:" << frameRate << std::endl;
		bFirst = false;
	}
	cv::Mat showMat = cv::Mat(cv::Size(pstFrame->width, pstFrame->height), CV_8UC3);
	if (pstFrame->type == AS_STREAM_DEPTH_IMAGE) {
		cv::Mat depth_data = cv::Mat(cv::Size(pstFrame->width, pstFrame->height), CV_16UC1, pstFrame->data);
		Depth2ColorMapTurbo(depth_data, showMat);
	}
	else if (pstFrame->type == AS_STREAM_DOT) {
		showMat = cv::Mat(cv::Size(pstFrame->width, pstFrame->height), CV_8UC1,pstFrame->data);
	}
	cv::imshow(std::string("camera: ") + std::to_string(deviceId), showMat);
	cv::waitKey(1);

	

#ifdef CFG_OPENCV_ON
	cv::Mat depth = cv::Mat(pstFrame->height, pstFrame->width, CV_16UC1, pstFrame->data);
	cv::Mat depth_img_pseudo_color;

	double minVal;
	double maxVal;
	cv::minMaxIdx(depth, &minVal, &maxVal);
	depth2color(depth_img_pseudo_color, depth, maxVal, minVal);
	cv::imshow("depth_" + std::to_string(0), depth_img_pseudo_color);
	cv::waitKey(3);
#endif
	
	if (s_bSaveFrameFlg)
	{
		s_CamPicFlag[deviceId] = true;
		
		static int depthindex = 0;
		static int pointCloudIndex = 0;
		int cloudPointSize = 0;

		std::string imgName(folderName + std::string("\\") + std::string("depth_480x640_" + std::to_string(deviceId) + "_" + std::to_string(
			depthindex) + ".yuv"));
		if (saveYUVImg(imgName.c_str(), pstFrame->data, pstFrame->size) != 0) {
			LOG(ERROR) << "save img failed!\n";
		}
		else {
			LOG(INFO) << "save depth image success!\n";
		}


		if (pstFrame->type != AS_STREAM_DOT)
		{
			float* pointCloud = new float[pstFrame->width * pstFrame->height * 3];
			if ((cloudPointSize = capture.convertDepthToCloudDot(deviceId, *pstFrame, pointCloud)) < 0) {
				LOG(ERROR) << "convert to cloudDot failed!\n";
				delete[]pointCloud;
				return;
			}
			std::string cloudDotName(folderName  + std::string("\\") + std::string("PointCloud" + std::to_string(deviceId) + "_" + std::to_string(
				pointCloudIndex) + ".txt"));
			if (saveCloudDot(cloudDotName.c_str(), pointCloud, cloudPointSize) != 0) {
				LOG(ERROR) << "save point cloud failed!\n";
			}
			else {
				LOG(INFO) << "save point cloud success!\n";
			}
			delete[]pointCloud;
		}


		for (int index = 0; index < deviceIDs.size(); index++)
		{
			if (s_CamPicFlag[index] == false)
			{
				return;
			}

		}

		
		s_bSaveFrameFlg = false;
		depthindex++;
		pointCloudIndex++;
		
		memset(s_CamPicFlag, 0, sizeof(bool)*deviceIDs.size());
		
	}
}


static void AS_SDK_PointCloudCallback(int deviceId, const AS_POINTCLOUD_Data* pstPointCloud, void* privateData)
{

}



static void* cmd_proc(void* arg)
{
	LOG(INFO) << "cmd_proc create success!\n";
	static int value = 0;
	Sleep(2000);
	while (1) {
		
		//capture.setCameraType(0, AS_STREAM_DOT);
		////capture.setVCSEL(0, 0);
		//capture.setCameraType(0, AS_STREAM_DEPTH_IMAGE);
		////capture.setVCSEL(0, 1);
		//continue;
		
		LOG(INFO) << "***********************************" << std::endl;
		LOG(INFO) << "1、press camera num the enter the control" << std::endl;
		LOG(INFO) << "2、press 'q' the exit the exe" << std::endl;
		LOG(INFO) << "***********************************" << std::endl;

		std::string cmd;
		std::string sub_cmd;
		std::cin >> cmd;
		if (cmd >= std::string("0") && cmd <= std::string("16"))
		{
			
			while (1)
			{
				LOG(INFO) << "controlling the camera: " << cmd << std::endl;
				LOG(INFO) << "***********************************" << std::endl;
				LOG(INFO) << "1、press '+' to increase the Gain" << std::endl;
				LOG(INFO) << "2、press '-' to increase the Gain" << std::endl;
				LOG(INFO) << "3、press '1' to open the VSCL" << std::endl;
				LOG(INFO) << "4、press '0' to close the VSCL" << std::endl;
				LOG(INFO) << "5、press 's' to save the image" << std::endl;
				LOG(INFO) << "6、press 'b' to switch the  IR image" << std::endl;
				LOG(INFO) << "7、press 'c' to switch the  depth image" << std::endl;
				LOG(INFO) << "8、press 'q' to exit the current camera control" << std::endl;
				LOG(INFO) << "***********************************" << std::endl;
				std::cin >> sub_cmd;
				if (sub_cmd >= std::string("0") && sub_cmd <= std::string("1"))
				{
					//LOG(INFO) << "Device: " << atoi(cmd.c_str()) << " gain: " << atoi(sub_cmd.c_str()) << std::endl;
					capture.setVCSEL(atoi(cmd.c_str()), atoi(sub_cmd.c_str()));
					
				}
				else if (sub_cmd == std::string("+"))
				{
					LOG(INFO) << "Camera Gain Mutiply: " << value++ << std::endl;;
					capture.setCameraGain(atoi(cmd.c_str()), value);
				}
				else if (sub_cmd == std::string("-"))
				{
					LOG(INFO) << "Camera Gain Mutiply: " << value-- << std::endl;;
					capture.setCameraGain(atoi(cmd.c_str()), value);
				}
				else if (sub_cmd == "b") {
					capture.setCameraType(atoi(cmd.c_str()), AS_STREAM_DOT);
				}
				else if (sub_cmd == "c") {
					capture.setCameraType(atoi(cmd.c_str()), AS_STREAM_DEPTH_IMAGE);
				}
				else if (sub_cmd == "s") {
					s_bSaveFrameFlg = true;
				}
				else if (sub_cmd == std::string("q"))
				{
					LOG(INFO) << "camera control exit!\n";
					break;
				}
			}
		}
		else if (cmd == std::string("-")) {
			AS_CAM_CFG_S stCamCfg;
			LOG(INFO) << "###########close TS" << std::endl;
			capture.getAsCamCfg(0, &stCamCfg);
			stCamCfg.isAtcOn = false;
			capture.setAsCamCfg(0, &stCamCfg);
		}
		else if (cmd == std::string("+")) {
			LOG(INFO) << "###########open TS" << std::endl;
			AS_CAM_CFG_S stCamCfg;
			capture.getAsCamCfg(0, &stCamCfg);
			stCamCfg.isAtcOn = true;
			capture.setAsCamCfg(0, &stCamCfg);
		}
		else if (cmd == "q") {
			break;
		}
		/*else if (cmd == "s") {
			s_bSaveFrameFlg = true;
		}*/
		else if (cmd == "b") {
			capture.setCameraType(0, AS_STREAM_DOT);
		}
		else if (cmd == "c") {
			capture.setCameraType(0, AS_STREAM_DEPTH_IMAGE);
		}
	}
	return 0;
}



int main()
{
	//CreateThread
	//openFileStream();
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	
	folderName = std::to_string(sys.wMonth) + std::to_string(sys.wDay) + std::string("_") \
		+ std::to_string(sys.wHour) + std::to_string(sys.wMinute);
	struct stat buffer;
	if (stat(folderName.c_str(), &buffer) != 0)
	{
		// if this folder not exist, create a new one.
		_mkdir(folderName.c_str());   // 返回 0 表示创建成功，-1 表示失败
		//换成 ::_mkdir  ::_access 也行，不知道什么意思
	}


	
	std::string fileName;
	fileName = folderName + std::string("\\") + std::to_string(sys.wMonth)  + std::to_string(sys.wDay) + std::string("_") \
		+ std::to_string(sys.wHour) + std::to_string(sys.wMinute) +std::string("day.txt");
	LOG(INFO) << fileName << std::endl;
	freopen(fileName.c_str(), "wb+", stdout);

	int ret = 0;
	std::string sdkVersion;
	//HRESULT hr = OleInitialize(NULL);
	LOG(INFO) << "Hello, angstrong NUWA camera" << std::endl;
	capture.getSDKSoftwareVersion(sdkVersion);
	LOG(INFO) << "Angstrong NuwaCam sdk version:" << sdkVersion << std::endl;
	std::vector<void*> test;
	int deviceNum;
	while (1)
	{
		Sleep(3000);
		deviceNum = capture.listDevice(deviceIDs);
		LOG(INFO) << "deviceNum: " << deviceNum << std::endl;
	/*	for (int i = 0; i < deviceNum; i++)
		{
			std::string sn_SN;
			capture.getSN(deviceIDs[i], sn_SN);
			LOG(INFO) << "sn[" << deviceIDs[i] << "]: " << sn_SN << std::endl;
		}
		Sleep(2000);
		}*/
		if (deviceNum <= 0) {
			LOG(INFO) << "no device!\n";
			return 0;
		}
		LOG(INFO) << "Found " << deviceNum << " nuwa camera" << std::endl;
		/*for (int index = 0; index <= deviceIDs.size(); index++)
		{
			LOG(INFO) << "deviceID[" << index << "]: " << deviceIDs.at(index) << std::endl;
		}*/

		s_CamPicFlag = new bool[deviceNum];
		memset(s_CamPicFlag, 0, sizeof(bool) * deviceNum);
		std::string sn_temp;
		capture.getSN(deviceIDs[0], sn_temp);
		LOG(INFO) << "sn :" << sn_temp << std::endl;

		for (int idx = 0; idx < deviceNum; idx++) 
		{
			AS_CAM_CFG_S stCamCfg;
			/*register callback function*/
			AS_DEPTHIMAGE_Callback stDepthImageCallback = {
				AS_SDK_DepthImageCallback,
			   (void*)0x1234,
			};

			AS_POINTCLOUD_Callback stPointCloudCallback = {
				(AS_PointCloudCallback)AS_SDK_PointCloudCallback,
				(void*)0x1234,
			};
#if 1
			/* register depth image data callback function*/
			capture.registerDepthImageCallback(deviceIDs.at(idx), &stDepthImageCallback);
			/* register point cloud data  callback function*/
			capture.registerPointCloudCallback(deviceIDs.at(idx), &stPointCloudCallback);


			/* open camera*/
			if (capture.openDevice(deviceIDs.at(idx)) != 0) {
				LOG(ERROR) << "device id " << deviceIDs.at(idx) << " open device failed!\n";
				continue;
			}
			LOG(INFO) << "Open camera " << deviceIDs.at(idx) << " success" << std::endl;


			/* get default camera parameter*/
			capture.getAsCamCfg(deviceIDs.at(idx), &stCamCfg);
			LOG(INFO) << " default camera cfg: Atc[" << stCamCfg.isAtcOn << "]" \
				<< " Denoises[" << stCamCfg.isDenoisesOn << "]" \
				<< " FillHoles[" << stCamCfg.isFillHolesOn << "]" \
				<< " AntiDistortion[" << stCamCfg.isAntiDistortion << "]" \
				<< std::endl;

			// stCamCfg.isAtcOn = false; /* Automatic Temperature Compensation, requires camera firmware support*/
			// stCamCfg.isDenoisesOn = true;
			// stCamCfg.isFillHolesOn = false;
			// stCamCfg.isAntiDistortion = false;
			/* config camera parameterc, if you don't call this method, it will use default parameter */
			// capture.setAsCamCfg(idx, &stCamCfg);

			/* set depth image frame rate, this feature requires camera firmware support.*/
			// capture.setFrameRate(idx, AS_FPS_15);

			if (capture.openStream(deviceIDs.at(idx)) != 0)
			{
				capture.closeDevice(deviceIDs.at(idx));
				LOG(ERROR) << "device id " << deviceIDs.at(idx) << " open stream failed!\n";
				continue;
			}
			LOG(INFO) << "Open Stream " << deviceIDs.at(idx) << " success" << std::endl;


			/* get camera serial number*/
			std::string SN;
			capture.getSN(deviceIDs.at(idx), SN);
			LOG(INFO) << "SN: " << SN << std::endl;
			s_bSaveFrameFlg = true;
			/* get camera parameter*/
			IrRgbParameter irRgbParameter;
			capture.getIrRgbParameter(deviceIDs.at(idx), &irRgbParameter);
			LOG(INFO) << "fx: " << irRgbParameter.irFx << std::endl;
			LOG(INFO) << "fy: " << irRgbParameter.irFy << std::endl;
			LOG(INFO) << "cx: " << irRgbParameter.irCx << std::endl;
			LOG(INFO) << "cy: " << irRgbParameter.irCy << std::endl;

			LOG(INFO) << "Camera Id " << deviceIDs.at(idx) << " start stream success" << std::endl;
			LOG(INFO) << "Enter 's' to save depth image and point cloud data" << std::endl;

			Sleep(3000);
			capture.closeStream(deviceIDs.at(idx));
			LOG(INFO) << "close Stream " << deviceIDs.at(idx)  << std::endl;
			capture.closeDevice(deviceIDs.at(idx));
			LOG(INFO) << "close Device " << deviceIDs.at(idx)  << std::endl;
			/*upgrade*/
			/*char data[512] = { 0 };
			FILE* fp = fopen(".bin","r");
			fread(data, sizeof(data), 1, fp);
			capture.upgradeHimax(0, data,sizeof(512));*/
			//capture.resetHimax(0);

			/*FILE* fp = fopen("aaa.txt", "wb+");
			if (fp == NULL) {
				LOG(ERROR) << "null ptr!\n";
				return -2;
			}
			fwrite("aaa", sizeof("aaa"), 1, fp);
			fflush(fp);
			fclose(fp);*/
			//fp.close();
			//_FSTREAM_ f("d:\\12.dat", ios::in | ios::out | ios::binary);
#endif
		}
	}
	/*create thread to process command*/	
	std::thread cmd_thread = std::thread(cmd_proc,(void *)NULL);
	cmd_thread.join();

	system("pause");
	return 0;
}