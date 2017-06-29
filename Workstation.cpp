#include "Workstation.h"


Workstation::Workstation():
currentDBNum(0)
{
}


Workstation::~Workstation()
{
}

void Workstation::addDashboard(Dashboard dashboard){
	dashboards.push_back(dashboard);
}

void Workstation::toggleDashboard(SwipeState swipeState){
	switch (swipeState)
	{
	case SwipeState_None:
		return;
		break;
	case SwipeState_Left:
		currentDBNum++;
		break;
	case SwipeState_Right:
		currentDBNum--;
		break;
	default:
		break;
	}

	if (currentDBNum < 0){
		currentDBNum = dashboards.size() - 1;
	}
	else if (currentDBNum > dashboards.size() - 1)
	{
		currentDBNum = 0;
	}

}

Dashboard* Workstation::getCurrentDashboard(){
	return &dashboards[currentDBNum];
}
