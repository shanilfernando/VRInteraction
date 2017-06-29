#include "stdafx.h"
#pragma once
#include <ctime>
#include <fstream>

#ifndef _SwipeState_
#define __SwipeState__
typedef enum _SwipeState SwipeState;

enum _SwipeState
{
	SwipeState_None = 0,
	SwipeState_Left = 1,
	SwipeState_Right = 2
};
#endif // _SwipeState_

#ifndef _Event__
#define __Event__
typedef enum _Event Event;

enum _Event
{
	EventNone = 0,
	KeyPressed = 1,
	KeyExceed = 2,
	KeyReleased = 3,
	KeyClicked= 4,
	KeyDoubleClicked= 5
};
#endif // _Event_

class Gesture
{
public:
	Gesture();
	~Gesture();
	float GetRotationAroundYAxis(CameraSpacePoint* joint1, CameraSpacePoint* joint2);
	float GetRotationAroundZAxis(CameraSpacePoint* joint1, CameraSpacePoint* joint2);
	float GetRotationAroundXAxis(CameraSpacePoint* joint1, CameraSpacePoint* joint2);
	float GetZoom(CameraSpacePoint* joint1, CameraSpacePoint* joint2);
	void setPreRightHand(CameraSpacePoint cameraSpacePoint);
	void setPreLeftHand(CameraSpacePoint cameraSpacePoint);
	SwipeState DetectSwipe(std::list<CameraSpacePoint>* points);
	bool DetectClick(CameraSpacePoint* hand, CameraSpacePoint* handTip);
	Eigen::Matrix3f GetRotationMatrix(float alpha, float beta, float gamma);
	Eigen::Quaternionf getQuaternion(CameraSpacePoint* joint1, CameraSpacePoint* joint2);
	bool isHandHorizontal(CameraSpacePoint* hand, CameraSpacePoint* handTip);
	vec3 Gesture::getRightHandDis(CameraSpacePoint* hand);
	

private:
	MVector preVectorXZ;
	MVector preVectorXY;
	MVector preVectorYZ;
	float rotationAroundYAxis=0.0f;
	float rotationAroundZAxis=0.0f;
	float rotationAroundXAxis=0.0f;
	CameraSpacePoint preRightHand;
	CameraSpacePoint preLeftHand;
	CameraSpacePoint preRightHandTip;
	Event currentEvent;
	Event preEvent;
	float getValue(float value);
	float getCurrentEz(float sz);
	float getCurrentSz(CameraSpacePoint* hand, CameraSpacePoint* handTip);
	float sensitivity;
	float weightH=3.0f;
	float weightHT=1.0f;
	float preSz = 0.0f;
	float preEz = 0.0f;
	float tau1 = -0.03f;
	float tau2 = -0.05f;
	float alpha = 1.0f;
	float midval = 0.5f;
	float threshold=0.5f;
	std::time_t preClickTime;
	std::ofstream myfile;


};

