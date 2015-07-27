/////////////////////// BUTTON /////////////////////////////

#include "VRPNButtonServer_IVW.h"

VRPNButtonServer_IVW::VRPNButtonServer_IVW(vrpn_Connection *c /*= 0 */, const char *name) :
vrpn_Button(name, c)
{
	// Setting the number of buttons to 10
	vrpn_Button::num_buttons = 3;

	vrpn_uint32 i;

	// initializing all buttons to false
	for (i = 0; i < (vrpn_uint32)vrpn_Button::num_buttons; i++) 
	{
		vrpn_Button::buttons[i] = vrpn_Button::lastbuttons[i] = 0;
	}
}

void VRPNButtonServer_IVW::mainloop()
{
	vrpn_gettimeofday(&_timestamp, NULL);
	vrpn_Button::timestamp = _timestamp;

	// forcing values to change otherwise vrpn doesn't report the changes
	static int b = 0; b++;

	for (unsigned int i = 0; i<vrpn_Button::num_buttons; i++)
	{
		// XXX Set your values here !
		buttons[i] = (ExternalData[i] + b) % 2;
	}

	// Send any changes out over the connection.
	vrpn_Button::report_changes();

	server_mainloop();
}


void VRPNButtonServer_IVW::CaptureData(vector <int> Data)
{
	ExternalData.clear();
	for (int i = 0; i < Data.size(); i++)
	{
		ExternalData.push_back(Data[i]);
	}

}