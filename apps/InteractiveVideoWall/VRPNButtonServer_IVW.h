/////////////////////// BUTTON /////////////////////////////

#ifndef INCLUDED_VRPNBUTTONSERVER_IVW_H
#define INCLUDED_VRPNBUTTONSERVER_IVW_H

#include <stdio.h>
#include <tchar.h>

#include <math.h>

#include "vrpn_Text.h"
#include "vrpn_Tracker.h"
#include "vrpn_Analog.h"
#include "vrpn_Button.h"
#include "vrpn_Connection.h"
#include <vector>
#include <iostream>
using namespace std;

// your button class must inherit from the vrpn_Button class
class VRPNButtonServer_IVW : public vrpn_Button
{
public:
	VRPNButtonServer_IVW(vrpn_Connection *c = 0, const char *name = "Button0");
	virtual ~VRPNButtonServer_IVW() {};

	virtual void mainloop();
	void CaptureData(vector <int> Data);

protected:
	vector<bool> ExternalData;
	struct timeval _timestamp;
};

#endif /*INCLUDED_VRPNBUTTONSERVER_IVW_H*/
