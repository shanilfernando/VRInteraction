#include "stdafx.h"
#include "Button.h"
#include <vector> 
#pragma once
class Dashboard
{
public:
	Dashboard();
	~Dashboard();

	void addButton(Button button);
	void update(CameraSpacePoint* position);
	void draw(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* hand);
	int getNumberPixel();
	void isClicked(CameraSpacePoint* hand);

private:
	std::vector<Button> buttons;
	float centerX;
	float centerY;
	float centerZ;
	float deltaX=0.2f;
	float deltaY=0.2f;
	float deltaZ = 0.30f;
	int numberButtons = 0;
};

