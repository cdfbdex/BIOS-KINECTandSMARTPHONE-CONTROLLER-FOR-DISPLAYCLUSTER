/////////////////////// ANALOG /////////////////////////////

// your analog class must inherin from the vrpn_Analog class

#include "VRPNAnalogServer_IVW.h"

VRPNAnalogServer_IVW::VRPNAnalogServer_IVW(vrpn_Connection *c, int numChannels, const char *name) :
vrpn_Analog(name, c)
{
	connectionName = name;

    vrpn_Analog::num_channel = numChannels;

    vrpn_uint32    i;

    for (i = 0; i < (vrpn_uint32)vrpn_Analog::num_channel; i++)
	{
        vrpn_Analog::channel[i] = vrpn_Analog::last[i] = 0;
		ExternalData.push_back(0);
    }
}

void VRPNAnalogServer_IVW::mainloop()
{
    vrpn_gettimeofday(&_timestamp, NULL);
    vrpn_Analog::timestamp = _timestamp;

    // forcing values to change between 0 and thresh value 
	// otherwise vrpn doesn't report the changes
    static float f = 0.0;
	
	float thresh = 0.1;
	if (f < thresh)
		f += 0.05;
	else
		f = 0.0;

    for( unsigned int i=0; i<vrpn_Analog::num_channel;i++)
    {
        // XXX Set your values here !
		//channel[i] = i / 10.f + f;
		//if (ExternalData[i]>0.2)
		if (i==0)
			channel[i] = double(ExternalData[i]+f);
		else
			channel[i] = double(ExternalData[i]);
    }
    // Send any changes out over the connection.
    vrpn_Analog::report_changes();

    server_mainloop();
}

void VRPNAnalogServer_IVW::CaptureData(vector <float> Data)
{
	ExternalData.clear();
	for (int i = 0; i < Data.size(); i++)
	{
		ExternalData.push_back(Data[i]);
	}

}

const char* VRPNAnalogServer_IVW::GetName()
{
	return connectionName;
}