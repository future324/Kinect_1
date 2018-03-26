#include"MyKinect.hpp"
int main()
{
	KinectManager km;
	while (waitKey(10)!=27)
	{
		km.BodyFrame();
		km.IrFrame();
		km.DepthFrame();
	}
	return 0;
}