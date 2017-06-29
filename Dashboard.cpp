#include "Dashboard.h"



Dashboard::Dashboard()
{
}


Dashboard::~Dashboard()
{
}

void Dashboard::addButton(Button button){
	buttons.push_back(button);
	numberButtons = buttons.size();
}

void Dashboard::update(CameraSpacePoint* position){
	double numButtons = buttons.size();
		for (int i = 0; i<numButtons; i++){
			buttons[i].update((position->X + deltaX + ((i % numberButtons) - 1)*(1.5*buttons[i].getButtonWidth())), -(position->Y + deltaY - (i / numberButtons)*(1.5*buttons[i].getButtonWidth())), (position->Z - deltaZ));
		}
	
}

int Dashboard::getNumberPixel(){
	return buttons.size() * 5 * 5;
}

void Dashboard::draw(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* hand){
	int numButtons = buttons.size();
	for (int i = 0; i<numButtons; i++){
		buttons[i].draw(start+25*i,cloud,hand);
	}
}

void Dashboard::isClicked(CameraSpacePoint* hand){
	double numButtons = buttons.size();
	for (int i = 0; i<numButtons; i++){
		bool result=buttons[i].isRightClicked(hand);
		if (result)
		{
			break;
		}
	}

}
