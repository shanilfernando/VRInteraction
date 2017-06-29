#include "User.h"


User::User():
leftHandFilter(KinectJointFilter()),
rightHandFilter(KinectJointFilter()),
rightHandTipFilter(KinectJointFilter()),
pointerFilter(KinectJointFilter()),
interactiveState(InteractiveState_Unknown),
workstation(new Workstation()),
gesture(new Gesture()),
interactionStart(false)
{
	ip.type = INPUT_KEYBOARD;
	ip.ki.wScan = 0; 
	ip.ki.time = 0;
	ip.ki.dwExtraInfo = 0;
	pointerJoint.JointType = JointType_HandRight;
	pointerJoint.TrackingState = TrackingState::TrackingState_Inferred;
}


User::~User()
{
	gesture->~Gesture();
}

CameraSpacePoint* User::getFilteredLeftHand(Joint joint){
	leftHandFilter.UpdateJoint(joint);
	HandLeft = leftHandFilter.GetFilteredJoint();
	return HandLeft;
}

CameraSpacePoint* User::getFilteredRightHand(Joint joint){
	rightHandFilter.UpdateJoint(joint);
	HandRight = rightHandFilter.GetFilteredJoint();
	return HandRight;
}

CameraSpacePoint* User::getFilteredRightHandTip(Joint joint){
	rightHandTipFilter.UpdateJoint(joint);
	rightHandTip = rightHandTipFilter.GetFilteredJoint();
	return rightHandTip;
}

CameraSpacePoint* User::getFilteredPointer(){
	pointerFilter.UpdateJoint(pointerJoint);
	pointer = pointerFilter.GetFilteredJoint();
	return pointer;
}

void User::toggleDashboard(){
	SwipeState swipeState = gesture->DetectSwipe(&rightHandTips);
	workstation->toggleDashboard(swipeState);
	switch (swipeState)
	{
	case SwipeState_Left:
		ip.ki.wVk = VK_LEFT;
		ip.ki.dwFlags = 0; 
		SendInput(1, &ip, sizeof(INPUT));
		break;
	case SwipeState_Right:
		ip.ki.wVk = VK_RIGHT;
		ip.ki.dwFlags = 0; 
		SendInput(1, &ip, sizeof(INPUT));
		break;
	default:
		break;
	}
}

void User::update(){
	workstation->getCurrentDashboard()->update(&spineMid);
}

void User::vizDashboard(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	cloud->points[start].x = HandRight->X;
	cloud->points[start].y = -HandRight->Y;
	cloud->points[start].z = HandRight->Z;
	cloud->points[start].r = 255;
	cloud->points[start].g = 255;
	cloud->points[start].b = 255;
	workstation->getCurrentDashboard()->draw(start+1, cloud, HandRight);
}

Eigen::Quaternionf User::getHandQuaternion(){
	return gesture->getQuaternion(HandLeft, HandRight);
}

float User::GetRotationAroundXAxis(){
	return gesture->GetRotationAroundXAxis(HandLeft, HandRight);
}

float User::GetRotationAroundYAxis(){
	return gesture->GetRotationAroundYAxis(HandLeft, HandRight);
}

float User::GetRotationAroundZAxis(){
	return gesture->GetRotationAroundZAxis(HandLeft, HandRight);
}

bool User::isRightHandHorizontal(){
	return gesture->isHandHorizontal(HandRight, rightHandTip);
}

bool User::isRightClicked(){
	bool result = gesture->DetectClick(HandRight, rightHandTip);
	if (result)
	{
		workstation->getCurrentDashboard()->isClicked(HandRight);
	}
	return result;
}

bool User::isMouseClicked(){
	bool result = gesture->DetectClick(HandRight, rightHandTip);
	return result;
}

