#ifndef XSensDriver_H
#define XSensDriver_H

#include <iostream>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <list>
#include <vector>
#include <xsens_api/cmtdef.h>
#include <xsens_api/xsens_time.h>
#include <xsens_api/xsens_list.h>
#include <xsens_api/cmtscan.h>
#include <xsens_api/cmt3.h>

using namespace std;

/************************************************************
 * Exception - Error
 ************************************************************/

class XSensDriverException: public std::exception {
private:
	const char* message;
	XsensResultValue result;
public:
	XSensDriverException (const char* what): message(what), result(XRV_OK) {	}
	XSensDriverException (const char* what, const XsensResultValue rslt): message(what), result(rslt) {}

	bool isResumable () {
		return this->result == XRV_INVALIDMSG ||
				this->result == XRV_TIMEOUT ||
				this->result == XRV_TIMEOUTNODATA;
	}

	virtual const char* what() const throw () {


		if(this->result == XRV_OK) return this->message;

		std::string msg(message);
		msg += " Error Code: ";
		msg += result;
		msg += " Error: ";
		msg += xsensResultText(result);
		return msg.c_str();
	}
};


class XSensDriver {
public:

	struct Mode {
		const static int quat = 0;
		const static int euler = 1;
		const static int cos_mat = 2;
	};

	struct Vector3 {
		double x;
		double y;
		double z;
	};

	XSensDriver();
	~XSensDriver();

	bool initialize();


	void fetchData();
	list<CmtScenario> availableScenarios ();
	CmtDeviceConfiguration* getConfiguration();
	int amountOfDevices();
	std::vector<CmtDeviceId> devicesIDS ();
	std::string strConfiguration();

	std::vector<Vector3*> acceleration;
	std::vector<Vector3*> gyroscope;
	std::vector<Vector3*> magneticField;
	std::vector<Vector3*> orientation;

private:

	void scanHardwareAndOpenEncounteredPorts();
	void configureDevices();
	void inline printIfIsNotOk(XsensResultValue value, const char* errorMessage);
	void explodeIfIsNotOk(XsensResultValue, const char*);

	uint8_t scenario;
	int timeOut;
	bool inited;

	CmtOutputSettings settings;
	CmtOutputMode mode;

	xsens::Cmt3 cmt3;
	xsens::List<CmtPortInfo> ports;
	xsens::Packet * packet;
};

#endif
