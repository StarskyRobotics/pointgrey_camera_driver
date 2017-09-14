/*
 This code was developed by the National Robotics Engineering Center (NREC), part
 of the Robotics Institute at Carnegie Mellon University.
 Its development was funded by DARPA under the LS3 program and submitted for
 public release on June 7th, 2012.
 Release was granted on August, 21st 2012 with Distribution Statement "A"
 (Approved for Public Release, Distribution Unlimited).

 This software is released under a BSD license:

 Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.
 Neither the name of the Carnegie Mellon University nor the names of its
 contributors may be used to endorse or promote products derived from this
 software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 @file list_cameras.cpp
 @author Chad Rockey
 @date January 10, 2012
 @brief Executable that lists the attached pointgrey cameras and exits.

 @attention Copyright (C) 2012
 @attention National Robotics Engineering Center
 @attention Carnegie Mellon University
 */

#include <stdio.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <string>
#include "pointgrey_camera_driver/PointGreyCamera.h"
//#include <socket.h>
#include <arpa/inet.h>
#include <map>
#include <unistd.h>

using namespace FlyCapture2;

#define PGERROR(error, msg) PointGreyCamera::handleError(msg, error)

std::string ip2str(const IPAddress& ip) {
	char str[INET_ADDRSTRLEN];
	sprintf(str, "%d.%d.%d.%d", ip.octets[0],ip.octets[1],ip.octets[2],ip.octets[3]);
	std::string ss = str;
	return ss;
}

void printCameraInfo(const CameraInfo& cinfo) {
	std::cout << "Serial: " << cinfo.serialNumber << ", Model: "
			<< cinfo.modelName << ", Vendor: " << cinfo.vendorName
			<< ", Sensor: " << cinfo.sensorInfo << ", Resolution: "
			<< cinfo.sensorResolution << ", Color: " << std::boolalpha
			<< cinfo.isColorCamera << ", Firmware Version: "
			<< cinfo.firmwareVersion << std::endl;
	std::cout << "  IP: " << ip2str(cinfo.ipAddress) << " Sub: "
			<< ip2str(cinfo.subnetMask) << " GW: "
			<< ip2str(cinfo.defaultGateway) << std::endl;
}

BusManager bus_manager;

void printAllCameraInfo() {
	std::cout << "Getting camera info..." << std::endl;
	try {
		unsigned num_devices = 0;
		PGERROR(bus_manager.GetNumOfCameras(&num_devices),
				"Failed get number of cameras");
		if (num_devices) {
			std::cout << "Number of cameras found: " << num_devices
					<< std::endl;
			for (unsigned i = 0; i < num_devices; ++i) {
				PGRGuid guid;
				std::ostringstream s;
				s << i;
				PGERROR(bus_manager.GetCameraFromIndex(i, &guid),
						"Failed to get camera from index " + s.str());

				Camera camera;
				PGERROR(camera.Connect(&guid), "Failed to connect to camera");

				CameraInfo cinfo;
				PGERROR(camera.GetCameraInfo(&cinfo),
						"Failed to get camera info");

				std::cout << "[" << i << "]";
				printCameraInfo(cinfo);
			}
		} else {
			// No cameras found
			std::cout << "No PointGrey cameras detected on this computer."
					<< std::endl << std::endl;

			std::cout << "Note that you may need to restart udev and "
					"replug your camera, eg:" << std::endl
					<< "  sudo service udev restart" << std::endl;
		}
	} catch (const std::runtime_error& e) {
		std::cout << "There was an error checking the active cameras: "
				<< e.what() << std::endl;
	}
}

void assignIPs() {
	std::cout << "Forcing IPs..." << std::endl;
	PGERROR(bus_manager.ForceAllIPAddressesAutomatically(),
			"Failed to force IPs");
    std::cout << "Sleeping..." << std::endl;
    sleep(3);
    std::cout << "Rescanning..." << std::endl;
	PGERROR(bus_manager.RescanBus(),
			"Failed to rescan");
}

#define REG_IP_ADDRESS 0x64c
#define REG_MASK 0x65c
#define REG_GATEWAY 0x66c

unsigned int ip2uint(IPAddress ip) {
	return (((unsigned int)ip.octets[0])<<24) 
        + (((unsigned int)ip.octets[1])<<16) 
        + (((unsigned int)ip.octets[2])<<8) 
        + (((unsigned int)ip.octets[3])<<0);
}

IPAddress uint2ip(unsigned int ui) {
	IPAddress ip;
	ip.octets[3] = ui & 0xff;
	ip.octets[2] = (ui>>8) & 0xff;
	ip.octets[1] = (ui>>16) & 0xff;
	ip.octets[0] = (ui>>24) & 0xff;
	return ip;
}

#define REG_NET 0x0014
#define NET_LLA (1<<29)
#define NET_DHCP (1<<30)
#define NET_PERSISTANT (1<<31)

void storeIPs() {
	std::cout << "Saving IPs..." << std::endl;
	try {
		unsigned num_devices = 0;
		PGERROR(bus_manager.GetNumOfCameras(&num_devices),
				"Failed get number of cameras");
		if (num_devices) {
			std::cout << "Number of cameras found: " << num_devices
					<< std::endl;
			for (unsigned i = 0; i < num_devices; ++i) {
				PGRGuid guid;
				std::ostringstream s;
				s << i;
				PGERROR(bus_manager.GetCameraFromIndex(i, &guid),
						"Failed to get camera from index " + s.str());

				GigECamera camera;
				PGERROR(camera.Connect(&guid), "Failed to connect to camera");

				CameraInfo cinfo;
				PGERROR(camera.GetCameraInfo(&cinfo),
						"Failed to get camera info");

				unsigned int ip_u, gw_u;

				camera.ReadGVCPRegister(REG_IP_ADDRESS, &ip_u);
				IPAddress ip = uint2ip(ip_u);

				camera.ReadGVCPRegister(REG_GATEWAY, &gw_u);
				IPAddress gw = uint2ip(gw_u);

                IPAddress gw_new = cinfo.ipAddress;
                gw_new.octets[3] = 1;

                IPAddress mask;
                mask.octets[0] = 0xff;
                mask.octets[1] = 0xff;
                mask.octets[2] = 0xff;
                mask.octets[3] = 0;

				std::cout << "[" << i << "]";
				std::cout << "IP register: " << ip2str(ip) << " GW: " << ip2str(gw) << std::endl;
                printf("IP reg hex: %08x %08x\n", ip_u, gw_u);
                std::cout << "Writing: " << ip2str(cinfo.ipAddress) << std::endl;
                printf("Writing hex: %08x %08x\n", ip2uint(cinfo.ipAddress), ip2uint(gw_new));


				camera.WriteGVCPRegister(REG_IP_ADDRESS, ip2uint(cinfo.ipAddress));
				camera.WriteGVCPRegister(REG_MASK, ip2uint(mask));
				camera.WriteGVCPRegister(REG_GATEWAY, ip2uint(gw_new));

                printf("Setting net reg to all\n");
				camera.WriteGVCPRegister(0x14, 0xffffffff);
                std::cout << "After writing: "<<std::endl;

				camera.ReadGVCPRegister(REG_IP_ADDRESS, &ip_u);
				ip = uint2ip(ip_u);

				camera.ReadGVCPRegister(REG_GATEWAY, &gw_u);
				gw = uint2ip(gw_u);

				std::cout << "[" << i << "]";
				std::cout << "IP register2: " << ip2str(ip) << " GW: " << ip2str(gw) << std::endl;
                printf("IP register2 hex: %08x %08x\n", ip_u, gw_u);
			}
		} else {
			// No cameras found
			std::cout << "No PointGrey cameras detected on this computer."
					<< std::endl << std::endl;

			std::cout << "Note that you may need to restart udev and "
					"replug your camera, eg:" << std::endl
					<< "  sudo service udev restart" << std::endl;
		}
	} catch (const std::runtime_error& e) {
		std::cout << "There was an error checking the active cameras: "
				<< e.what() << std::endl;
	}
}

int main(int argc, char** argv) {
	printAllCameraInfo();
	assignIPs();
	storeIPs();
	printAllCameraInfo();

	return 0;
}
