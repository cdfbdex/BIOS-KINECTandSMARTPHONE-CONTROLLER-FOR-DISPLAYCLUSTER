/////////////////////// ANALOG /////////////////////////////

// your analog class must inherin from the vrpn_Analog class
#ifndef INCLUDED_VRPNANALOGSERVER_IVW_H
#define INCLUDED_VRPNANALOGSERVER_IVW_H

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


class VRPNAnalogServer_IVW : public vrpn_Analog
{
public:
	VRPNAnalogServer_IVW(vrpn_Connection *c = 0, int numChannels = 1, const char *name = "Analog0");
	virtual ~VRPNAnalogServer_IVW() {};

    virtual void mainloop();
	void CaptureData(vector <float> Data);
	const char* GetName();

protected:
	const char* connectionName;
	vector<float > ExternalData;
    struct timeval _timestamp;
};


#endif /*INCLUDED_VRPNANALOGSERVER_IVW_H*/