#pragma once

#include <string>

// Reads the USB vendor and product ids out of a device path, which carries
// them as "usb#vid_046d&pid_082d". False for a device that is not USB.
bool parseUsbDeviceIds(const std::string& devicePath, unsigned int& outVendorId, unsigned int& outProductId);
