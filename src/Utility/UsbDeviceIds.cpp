#include "UsbDeviceIds.h"

#include <algorithm>
#include <cctype>
#include <cstdio>

bool parseUsbDeviceIds(const std::string& devicePath, unsigned int& outVendorId, unsigned int& outProductId)
{
	std::string lowerPath= devicePath;
	std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
				   [](unsigned char c) { return (char)std::tolower(c); });

	const size_t idsStart= lowerPath.find("usb#vid_");
	if (idsStart == std::string::npos)
		return false;

	unsigned int vendorId= 0;
	unsigned int productId= 0;
	if (sscanf_s(lowerPath.c_str() + idsStart, "usb#vid_%x&pid_%x", &vendorId, &productId) != 2)
		return false;

	outVendorId= vendorId;
	outProductId= productId;
	return true;
}
