#include "stdafx.h"
#include "resource.h"
#pragma once
class Button
{
	typedef int(*CallbackType)(Button);
public:
	Button(std::string id,CallbackType callback);
	Button::Button(std::string id,CallbackType callback, double buttonWidth);
	~Button();
	void draw(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* hand);
	bool isOnButton(CameraSpacePoint* hand);
	bool isRightClicked(CameraSpacePoint* hand);
	void update(float x, float y, float z);
	int getNumberOfPoints();
	std::string getId(){ return id; };
	double getButtonWidth(){ return buttonWidth / 100; };
	

private:
	double buttonWidth;
	float centerX;
	float centerY;
	float centerZ;
	CallbackType regCallback;
	void callback();
	std::string id;
};

