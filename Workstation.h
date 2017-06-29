#pragma once
#include <vector>
#include "Dashboard.h"
#include "Gesture.h"
class Workstation
{
public:
	Workstation();
	~Workstation();
	void addDashboard(Dashboard dashboard);
	void toggleDashboard(SwipeState swipeState);
	Dashboard* getCurrentDashboard();
	int getNumDashboards(){ return dashboards.size(); };

private:
	std::vector<Dashboard> dashboards;
	int currentDBNum;
};

