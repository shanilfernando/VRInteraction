#pragma once
#include "Gesture.h"
#include "KinectJointFilter.h"
#include "Workstation.h"
#include "Gesture.h"

#ifndef _InteractiveState_
#define _InteractiveState_
typedef enum _InteractiveState InteractiveState;
enum _InteractiveState
{
	InteractiveState_Unknown = 0,
	InteractiveState_Rotate = 1,
	InteractiveState_Zoom = 2,
	InteractiveState_Tracked = 3,
	InteractiveState_Swipe = 4,
	InteractiveState_Click = 5	
};
#endif 

class User
{
public:
	User();
	~User();
	UINT64 userId;
	bool isTracked;
	bool interactionStart;
	CameraSpacePoint* getFilteredLeftHand(Joint joint);
	CameraSpacePoint* getFilteredRightHand(Joint joint);
	CameraSpacePoint* getFilteredRightHandTip(Joint joint);
	CameraSpacePoint* getFilteredPointer();
	CameraSpacePoint* HandRight;
	CameraSpacePoint* HandLeft;
	CameraSpacePoint spineMid;
	CameraSpacePoint* rightHandTip;
	InteractiveState interactiveState;
	std::list<CameraSpacePoint> rightHandTips;
	void update();
	Workstation* workstation;
	Gesture* gesture;
	void toggleDashboard();
	void vizDashboard(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	Eigen::Quaternionf getHandQuaternion();
	float GetRotationAroundXAxis();
	float GetRotationAroundYAxis();
	float GetRotationAroundZAxis();
	HandState preRightHandState;
	bool isRightHandHorizontal();
	bool isRightClicked();
	CameraSpacePoint* pointer;
	bool isMouseClicked();
	Joint pointerJoint;

private:
	KinectJointFilter leftHandFilter;
	KinectJointFilter rightHandFilter;
	KinectJointFilter rightHandTipFilter;
	KinectJointFilter pointerFilter;
	INPUT ip;
	
};

