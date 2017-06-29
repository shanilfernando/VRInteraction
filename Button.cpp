#include "Button.h"


Button::Button(std::string id,CallbackType callback) :
buttonWidth(5.0),
id(id)
{
	regCallback = callback;
}

Button::Button(std::string id,CallbackType callback, double buttonWidth) :
buttonWidth(buttonWidth),
id(id)
{
	regCallback = callback;
}


Button::~Button()
{
}

void Button::draw(int start, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CameraSpacePoint* hand){
	int n = 0;
	
	if (isOnButton(hand)){
		for (double i = 0; i < buttonWidth; i++){
			for (double j = 0; j < buttonWidth; j++)
			{
				cloud->points[start + n].x = centerX + j / 100;
				cloud->points[start + n].y = (centerY + i / 100);
				cloud->points[start + n].z = centerZ;
				cloud->points[start + n].r = 0;
				cloud->points[start + n].g = 0;
				cloud->points[start + n].b = 255;
				n++;
			}
		}
	}
	else{
		
		for (double i = 0; i < buttonWidth; i++){
			for (double j = 0; j < buttonWidth; j++)
			{
				
				cloud->points[start + n].x = centerX + j / 100;
				cloud->points[start + n].y = (centerY + i / 100);
				cloud->points[start + n].z = centerZ;
				cloud->points[start + n].r = 255;
				cloud->points[start + n].g = 255;
				cloud->points[start + n].b = 255;
				n++;
			}
		}
	}
}

bool Button::isOnButton(CameraSpacePoint* hand){
	bool result = false;
	if (centerX <= hand->X && hand->X <= centerX + 0.05 && centerY - 0.025 <= -hand->Y && -hand->Y <= centerY + 0.025){
		result = true;
	}
	return result;
}

void Button::update(float x,float y,float z){
	centerX = x;// -buttonWidth / 200;
	centerY = y;// -buttonWidth / 200;
	centerZ = z;

}

int Button::getNumberOfPoints(){
	return buttonWidth*buttonWidth;
}

void Button::callback(){
	regCallback(*this);
}

bool Button::isRightClicked(CameraSpacePoint* hand){
	bool result = isOnButton(hand);
	if (result)
	callback();
	return result;
}
