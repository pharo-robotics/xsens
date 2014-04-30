/*******************************************************************************
 *
 *      @Author Santiago Bragagnolo
 *
 *******************************************************************************/


#include <xsens_driver/xsens_driver.h>

#include <iostream>
#include <sstream>
#include <memory>

using namespace xsens;
using namespace std;

XSensDriver::XSensDriver() :
		scenario(6),
		timeOut(100),
		inited(false),
		settings(CMT_OUTPUTSETTINGS_ORIENTMODE_EULER),

		mode(CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT),
		packet(NULL) {
	if (!this->initialize()){
		throw XSensDriverException ("Error initializing driver");
	}
}


void inline releaseMemoryFromVector (std::vector<XSensDriver::Vector3*> & vector){
	for (std::vector<XSensDriver::Vector3*>::iterator it = vector.begin() ; it != vector.end(); ++it){
		delete *it;
	}
	vector.clear();
}

XSensDriver::~XSensDriver() {

	releaseMemoryFromVector(acceleration);
	releaseMemoryFromVector(gyroscope);
	releaseMemoryFromVector(magneticField);
	releaseMemoryFromVector(orientation);

	cmt3.closePort();
	delete packet;

}

void inline XSensDriver::explodeIfIsNotOk(XsensResultValue value, const char* errorMessage){
	if (value != XRV_OK) {
		throw XSensDriverException(errorMessage, value);
	}
}


list<CmtScenario> XSensDriver::availableScenarios () {
	CmtScenario scenarios[CMT_MAX_SCENARIOS_IN_MT + 1];
	list<CmtScenario> filteredScenarios;
	this->explodeIfIsNotOk(cmt3.getAvailableScenarios(scenarios), "Error obtaining available scenarios");
	for(int i = 0; scenarios[i].m_type != 0 and i < CMT_MAX_SCENARIOS_IN_MT + 1; i ++) filteredScenarios.push_back(scenarios[i]);
	return filteredScenarios;
}


bool XSensDriver::initialize() {

	if (inited) return true;

	this->scanHardwareAndOpenEncounteredPorts();

	int devices = this->amountOfDevices();

	if(devices == 0) return false;


	inited = true;

	this->explodeIfIsNotOk(cmt3.setTimeoutMeasurement(timeOut), "Error setting measurement timeout");


	try {
		this->explodeIfIsNotOk(cmt3.setScenario(scenario), "Error setting scenario");
	} catch (XSensDriverException & e){
		if(!e.isResumable()) throw e;
	}

	this->configureDevices();

	packet = new Packet((unsigned short) devices, cmt3.isXm());



	this->acceleration.reserve(devices);
	this->gyroscope.reserve(devices);
	this->magneticField.reserve(devices);
	this->orientation.reserve(devices);

	for(int i = 0; i < devices ; i++) {
		this->acceleration.push_back(new Vector3);
		this->gyroscope.push_back(new Vector3);
		this->magneticField.push_back(new Vector3);
		this->orientation.push_back(new Vector3);
	}


	return true;
}


CmtDeviceConfiguration* XSensDriver::getConfiguration() {
	CmtDeviceConfiguration* configuration = new CmtDeviceConfiguration;
	this->explodeIfIsNotOk(cmt3.getConfiguration(*configuration), "Error getting configuration");
	return configuration;
}

string XSensDriver::strConfiguration() {
	CmtDeviceConfiguration* configuration = this->getConfiguration();
	ostringstream conf;
	conf << "Configuration:" << std::endl;
	for (uint16_t devIdx = 0; devIdx < configuration->m_numberOfDevices; devIdx++) {
		conf << " - Device[" << devIdx << "]" << std::endl;
		conf << " |- m_currentScenario:  " << configuration->m_deviceInfo[0].m_currentScenario << std::endl;
		conf << " |- m_filterType:       " << (int) configuration->m_deviceInfo[0].m_filterType << std::endl;
		conf << " |- m_filterMajor:      " << (int) configuration->m_deviceInfo[0].m_filterMajor << std::endl;
		conf << " |- m_filterMinor:      " << (int) configuration->m_deviceInfo[0].m_filterMinor << std::endl;
	}
	delete configuration;
	return conf.str();
}

void XSensDriver::fetchData() {
	if (!inited) throw XSensDriverException("Error Driver not initialized (call initialize before start using)");

	this->explodeIfIsNotOk(cmt3.waitForDataMessage(packet), "Error waiting for message");


	CmtCalData caldata;
	CmtEuler euler;

	for( int device = 0; device < this->amountOfDevices(); device ++){

		caldata = packet->getCalData(device);

		acceleration[device]->x = caldata.m_acc.m_data[0];
		acceleration[device]->y = caldata.m_acc.m_data[1];
		acceleration[device]->z = caldata.m_acc.m_data[2];

		gyroscope[device]->x = caldata.m_gyr.m_data[0];
		gyroscope[device]->y = caldata.m_gyr.m_data[1];
		gyroscope[device]->z = caldata.m_gyr.m_data[2];

		magneticField[device]->x = caldata.m_mag.m_data[0];
		magneticField[device]->y = caldata.m_mag.m_data[1];
		magneticField[device]->z = caldata.m_mag.m_data[2];


		euler = packet->getOriEuler(device);

		orientation[device]->x = euler.m_roll;
		orientation[device]->y = euler.m_pitch;
		orientation[device]->z = euler.m_yaw;
	}

}


void XSensDriver::scanHardwareAndOpenEncounteredPorts() {
	xsens::cmtScanPorts(ports);

	printf("scanned ports %d\n", ports.length());


	if (ports.length() != 0) {
		for(int p = 0; p < (int) ports.length(); p++){
			this->explodeIfIsNotOk(cmt3.openPort(ports[p].m_portName, ports[p].m_baudrate), "Error opening port");
		}
	}

}


vector<CmtDeviceId> XSensDriver::devicesIDS () {
	vector<CmtDeviceId> deviceIds;
	CmtDeviceId swaper;
	deviceIds.reserve(10);

	for(int j = 0; j < this->amountOfDevices(); j++){
		this->explodeIfIsNotOk(cmt3.getDeviceId((unsigned char)(j+1), swaper), "Error obtaining device ID");
		deviceIds.push_back(swaper);
	}

	return deviceIds;
}

int XSensDriver::amountOfDevices() {
	if (ports.length() == 0) return 0;
	return cmt3.getMtCount();
}



void XSensDriver::configureDevices() {
	this->explodeIfIsNotOk(cmt3.gotoConfig(), "Error Beginning configuration");

	vector<CmtDeviceId> deviceIds = this->devicesIDS() ;

	for (vector<CmtDeviceId>::iterator it = deviceIds.begin() ; it != deviceIds.end(); it ++) {
		CmtDeviceMode deviceMode(mode, settings, cmt3.getSampleFrequency());
		// not an MTi-G, remove all GPS related stuff
		if ((*it & 0xFFF00000) != 0x00500000) { deviceMode.m_outputMode &= 0xFF0F; }
		this->explodeIfIsNotOk(cmt3.setDeviceMode(deviceMode, true, *it), "Error setting device mode");
	}

	this->explodeIfIsNotOk(cmt3.gotoMeasurement(), "Error beginning measurement mode");
}
























