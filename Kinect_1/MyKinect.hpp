#include<opencv2\opencv.hpp>
#include<Kinect.h>
using namespace std;
using namespace cv;
#define Devview(i) imshow(#i,i)

void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)
{
	//用两个关节点来做线段的两端，并且进行状态过滤  
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点  
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;

		line(img, p_1, p_2, Vec3b(0, 255, 0), 5);
		circle(img, p_1, 10, Vec3b(255, 0, 0), -1);
		circle(img, p_2, 10, Vec3b(255, 0, 0), -1);
	}
}

class KinectManager
{
public:
	KinectManager();
	~KinectManager();
	void colorframe();
	void IrFrame();
	void DepthFrame();
	void BodyFrame();
private:
	int cheight, iheight, dheight = 0;
	int cwidth, iwidth, dwidth = 0;
	int body_count = 0;

	IKinectSensor * my_KinectSensor;
	IColorFrameSource* pColorSource = nullptr;
	IFrameDescription *pcolorDescription = nullptr;//获得分辨率
	IColorFrameReader *pcolorReader = nullptr;
	IColorFrame *pcolorFrame = nullptr;

	IInfraredFrameSource* pInfraredSource = nullptr;
	IFrameDescription *pInfraredDescription = nullptr;
	IInfraredFrameReader *pInfraredReader = nullptr;
	IInfraredFrame *pInfraredFrame = nullptr;

	IDepthFrameSource *pDepthSource = nullptr;
	IDepthFrameReader *pDepthReader = nullptr;
	IFrameDescription *pDepthDescription = nullptr;
	IDepthFrame *pDepthFrame = nullptr;

	IBodyFrameSource *pBodySource = nullptr;
	IBodyFrameReader *pBodyReader = nullptr;
	ICoordinateMapper *myMapper = nullptr;
	IBodyFrame* pBodyframe = nullptr;
	
};

KinectManager::KinectManager()
{
	HRESULT hr = GetDefaultKinectSensor(&my_KinectSensor);//获取设备
	
	my_KinectSensor->Open();

	my_KinectSensor->get_ColorFrameSource(&pColorSource);// 获取彩色帧源(ColorFrameSource)
	pColorSource->get_FrameDescription(&pcolorDescription);
	pcolorDescription->get_Height(&cheight);
	pcolorDescription->get_Width(&cwidth);
	pcolorDescription->Release();
	pColorSource->OpenReader(&pcolorReader);//打开彩色帧reader

	my_KinectSensor->get_DepthFrameSource(&pDepthSource);// 获取深度
	pDepthSource->get_FrameDescription(&pDepthDescription);
	pDepthDescription->get_Height(&dheight);
	pDepthDescription->get_Width(&dwidth);
	pDepthDescription->Release();
	pDepthSource->OpenReader(&pDepthReader);//打开深度帧

	my_KinectSensor->get_InfraredFrameSource(&pInfraredSource);// 获取红外
	pInfraredSource->get_FrameDescription(&pInfraredDescription);
	pInfraredDescription->get_Height(&iheight);
	pInfraredDescription->get_Width(&iwidth);
	pInfraredDescription->Release();
	pInfraredSource->OpenReader(&pInfraredReader);//打开红外帧

	my_KinectSensor->get_BodyFrameSource(&pBodySource);
	pBodySource->OpenReader(&pBodyReader);
	my_KinectSensor->get_CoordinateMapper(&myMapper);
	pBodySource->get_BodyCount(&body_count);
	
}

KinectManager::~KinectManager()
{
	pColorSource->Release();
	pcolorReader->Release();
	my_KinectSensor->Close();
	my_KinectSensor->Release();
}
void KinectManager::colorframe()
{
	Mat color(cheight, cwidth, CV_8UC4);
	if (pcolorReader->AcquireLatestFrame(&pcolorFrame) == S_OK)
	{
		pcolorFrame->CopyConvertedFrameDataToArray(8294400, (BYTE*)color.data, ColorImageFormat_Bgra);
		cv::pyrDown(color, color);
		Devview(color);
		pcolorFrame->Release();
	}

}
void KinectManager::IrFrame()
{
	Mat temp(iheight, iwidth, CV_16UC1);
	Mat IrFrame(iheight, iwidth, CV_8UC1);
	if (pInfraredReader->AcquireLatestFrame(&pInfraredFrame) == S_OK)
	{
		pInfraredFrame->CopyFrameDataToArray(iheight*iwidth, (UINT16*)temp.data);
		temp.convertTo(IrFrame, CV_8UC1, 255.0 / 4500);
		Devview(IrFrame);
		pInfraredFrame->Release();
	}

}
void KinectManager::DepthFrame()
{
	Mat temp(dheight, dwidth, CV_16UC1);
	Mat depth(dheight, cwidth, CV_8UC1);
	if (pDepthReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
	{
		pDepthFrame->CopyFrameDataToArray(dheight*dwidth, (UINT16*)temp.data);
		temp.convertTo(depth, CV_8UC1, 255.0 / 4500);
		Devview(depth);
		pDepthFrame->Release();
	}
}
void  KinectManager::BodyFrame()
{
	Mat colorFrame(cheight, cwidth, CV_8UC4);
	IBody* myBodyArr[BODY_COUNT] = { 0 };//每一个IBody可以追踪一个人，总共可以追踪六个人
	while (pcolorReader->AcquireLatestFrame(&pcolorFrame) != S_OK);
	
		if (pcolorFrame!=NULL)
		{
			pcolorFrame->CopyConvertedFrameDataToArray(8294400, (BYTE*)colorFrame.data, ColorImageFormat_Bgra);
			pcolorFrame->Release();
		}
		
	
	if (pBodyReader->AcquireLatestFrame(&pBodyframe) == S_OK&&pBodyframe->GetAndRefreshBodyData(body_count, myBodyArr) == S_OK)
	{
		Mat copy = colorFrame.clone();        //读取彩色图像并输出到矩阵  
		for (int i = 0; i < body_count; i++)
		{
			BOOLEAN     result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到  
			{
				Joint   myJointArr[JointType_Count];
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   //如果侦测到就把关节数据输入到数组并画图  
				{
					draw(copy, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper);
					draw(copy, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper);

					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper);
					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper);
					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper);

					draw(copy, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper);
					draw(copy, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper);
					draw(copy, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper);

					draw(copy, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper);
					draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper);
					draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper);
					draw(copy, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper);

					draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper);
					draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper);
					draw(copy, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper);
					draw(copy, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper);
					draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper);
					draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper);

					draw(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper);
					draw(copy, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper);
					draw(copy, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper);
					draw(copy, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper);
				}
			}
			
		}
		
		for (size_t i = 0; i < BODY_COUNT; i++)
		{
			myBodyArr[BODY_COUNT] = { 0 };
		}
		Devview(copy);
		pBodyframe->Release();
	
	}
}