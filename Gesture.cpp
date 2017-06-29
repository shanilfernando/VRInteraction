#include "Gesture.h"


Gesture::Gesture():
preVectorXY(MVector()),
preVectorXZ(MVector()),
rotationAroundYAxis(0.0f),
rotationAroundZAxis(0.0f),
rotationAroundXAxis(0.0f),
preLeftHand(CameraSpacePoint()),
preRightHand(CameraSpacePoint()),
sensitivity(1.0f)
{
	myfile.open("Click.xml");
	myfile << "<Data>\n";
}


Gesture::~Gesture()
{
	myfile << "</Data>\n";
	myfile.close();
}

Eigen::Matrix3f Gesture::GetRotationMatrix(float alpha, float beta, float gamma){
	float s_a = sinf(alpha);
	float c_a = cosf(alpha);

	float s_b = sinf(beta);
	float c_b = cosf(beta);

	float s_g = sinf(gamma);
	float c_g = cosf(gamma);
	Eigen::Matrix3f Rx;
	Rx <<	1,	0,		0,
			0,	c_a,	-s_a,
			0,	s_a,	c_a	;
	Eigen::Matrix3f Ry;
	Ry <<	c_b,	0,	s_b,
			0,		1,	0,
			-s_b,	0,	c_b;
	Eigen::Matrix3f Rz;
	Rz <<	c_g,	-s_g,	0,
			s_g,	c_g,	0,
			0,		0,		1;

	Eigen::Matrix3f rotaionMatrix = Rz*Ry*Rx;
	return rotaionMatrix;
}

Eigen::Quaternionf Gesture::getQuaternion(CameraSpacePoint* joint1, CameraSpacePoint* joint2){
	float alpha = 0.0f;// GetRotationAroundXAxis(joint1,joint2);
	float beta = GetRotationAroundYAxis(joint1,joint2);
	float gamma = GetRotationAroundZAxis(joint1,joint2);
	Eigen::Matrix3f rotaionMatrix = GetRotationMatrix(alpha, beta,gamma);
	Eigen::Quaternionf quaternion(rotaionMatrix);
	return quaternion;
}

float Gesture::GetRotationAroundYAxis(CameraSpacePoint* joint1,CameraSpacePoint* joint2){

	MVector newVector = MVector(joint1->X, joint1->Z, joint2->X, joint2->Z);
	float angle = -MVector::GetAngleBetweenTwoVectors(preVectorXZ, newVector);
	rotationAroundYAxis = rotationAroundYAxis + angle * sensitivity;
	preVectorXZ = newVector;
	return rotationAroundYAxis;
}

float Gesture::GetRotationAroundZAxis(CameraSpacePoint* joint1, CameraSpacePoint* joint2){

	MVector newVector = MVector(joint1->X, joint1->Y, joint2->X, joint2->Y);
	float angle = -MVector::GetAngleBetweenTwoVectors(preVectorXY, newVector);
	rotationAroundZAxis = rotationAroundZAxis + angle * sensitivity;
	preVectorXY=newVector;
	return rotationAroundZAxis;
}

float Gesture::GetRotationAroundXAxis(CameraSpacePoint* joint1, CameraSpacePoint* joint2){

	MVector newVector = MVector(joint1->Y, joint1->Z, joint2->Y, joint2->Z);
	float angle = -MVector::GetAngleBetweenTwoVectors(preVectorYZ, newVector);
	rotationAroundXAxis = rotationAroundXAxis + angle * sensitivity;
	preVectorYZ = newVector;
	return rotationAroundXAxis;
}

SwipeState Gesture::DetectSwipe(std::list<CameraSpacePoint>* points)
{
	
	SwipeState swipeState = SwipeState_None;
	float MinXDelta = 0.2f; 
	float MaxYDelta = 0.1f; 
	float threshhold = 0.01f;

	float x1 = points->front().X;
	float y1 = points->front().Y;
	float x2 = points->back().X;
	float y2 = points->back().Y;
	if (abs(x1 - x2) < MinXDelta)
		return swipeState;
	
	if (abs(y1 - y2) > MaxYDelta)
		return swipeState;
	float constant = (x1 * y2 - x2 * y1);
	/*std::ostringstream ss;*/
	bool direction = signbit(x2-x1);
	std::list<CameraSpacePoint>::iterator preit = points->begin();
	for (std::list<CameraSpacePoint>::iterator it = ++points->begin(); it != points->end(); preit = it, it++)
	{
		
		if (abs((it->Y - y1)) > MaxYDelta)
			return swipeState;
		

		if(signbit(it->X - preit->X) != direction)
			return swipeState;

		float result =
			(y1 - y2) * it->X+
			(x2 - x1) * it->Y +
			constant;
		/*ss << "<RighthandTip>\n";
		ss << "<Xabs>" << abs(x1 - x2) << "</Xabs>\n";
		ss << "<Yabs>" << abs(y1 - y2) << "</Yabs>\n";
		ss << "<Y2abs>" << abs(it->Y - y1) << "</Y2abs>\n";
		ss << "<result>" << result << "</result>\n";
		ss << "<signbit>" << direction << "</signbit>\n";
		ss << "</RighthandTip>\n";*/
		
		if (abs(result) > threshhold)
		{
			return swipeState;
		}
	}
	/*myfile << ss.str();*/
	
	
	if (direction)
	{
		swipeState = SwipeState_Left;
		points->clear();
	}
	else{
		swipeState = SwipeState_Right;
		points->clear();
	}
	return swipeState;
}

bool Gesture::DetectClick(CameraSpacePoint* hand, CameraSpacePoint* handTip)
{
	bool result=false;
	if (hand->Z>0.0f){
		
		myfile << "<Click>\n";
		myfile << "<hand>" << hand->Z << "</hand>\n";
		myfile << "<handTip>" << handTip->Z << "</handTip>\n";

		float sz = getCurrentSz(hand,handTip);
		float ez = getCurrentEz(sz);

		myfile << "<Sz>" << sz << "</Sz>\n";
		myfile << "<Ez>" << ez << "</Ez>\n";
		if (handTip->Z <= sz && ez <= handTip->Z && preSz < preRightHandTip.Z){
			currentEvent = KeyPressed;
		}
		else if (handTip->Z < ez && preRightHandTip.Z <= preSz && preEz <= preRightHandTip.Z){
			currentEvent = KeyExceed;
		}
		else if (preRightHandTip.Z <= preSz && preEz <= preRightHandTip.Z && sz < handTip->Z){
			currentEvent = KeyReleased;
			if (preEvent == KeyPressed){
				currentEvent = KeyClicked;
				std::time_t now = std::time(0);
				if ((now - preClickTime) < 1){
					currentEvent = KeyDoubleClicked;
				}
				preClickTime = now;
				result = true;
				std::cout << currentEvent << std::endl;
			}
		}
		preRightHand = *hand;
		preRightHandTip = *handTip;
		preSz = sz;
		preEz = ez;
		preEvent = currentEvent;
		myfile << "</Click>\n";
	}
	return result;
}

float Gesture::GetZoom(CameraSpacePoint* joint1, CameraSpacePoint* joint2){

	return ((getValue(joint1->Z) - getValue(preLeftHand.Z) + getValue(joint2->Z) - getValue(preRightHand.Z)) / 2);

}

float Gesture::getValue(float value){

	return roundf(value * 100);

}

float Gesture::getCurrentEz(float sz){
	float ratio = (std::abs(alpha - midval) / 0.5);
	if (ratio<threshold){
		ratio = 0.0f;
	}
	float con = 1 - ratio;
	/*myfile << "<con>" << con << "</con>\n";*/
	return (sz + tau2*con);
}

float Gesture::getCurrentSz(CameraSpacePoint* hand,CameraSpacePoint* handTip){
	
	float ave = (handTip->Z*weightHT + hand->Z*weightH) / (weightH + weightHT);
	myfile << "<ave>" << ave << "</ave>\n";
	float A = (hand->X - handTip->X)*(hand->X - handTip->X) + (hand->Y - handTip->Y)*(hand->Y - handTip->Y);
	float B = (hand->Z - handTip->Z)*(hand->Z - handTip->Z);
	alpha = (A / (A + B));
	/*myfile << "<alpha>" << alpha << "</alpha>\n";*/
	return (ave + tau1*alpha);
}

void Gesture::setPreRightHand(CameraSpacePoint cameraSpacePoint){
	preRightHand = cameraSpacePoint;
}

void Gesture::setPreLeftHand(CameraSpacePoint cameraSpacePoint){
	preLeftHand = cameraSpacePoint;
}

bool Gesture::isHandHorizontal(CameraSpacePoint* hand, CameraSpacePoint* handTip){
	bool result = false;
	if (abs(hand->Y - handTip->Y)<0.05f){
		result = true;
	}
	return result;
}

vec3 Gesture::getRightHandDis(CameraSpacePoint* hand){
	vec3 result;
	result.x = hand->X - preRightHand.X;
	result.y = hand->Y - preRightHand.Y;
	result.z = hand->Z - preRightHand.Z;
	return result;
}


