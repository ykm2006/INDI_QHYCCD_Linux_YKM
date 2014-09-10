#ifndef INDI_QHYCCD_LINUX_H
#define INDI_QHYCCD_LINUX_H

/*
 QHY CCD INDI Driver based on QHYCCD_Linux

 Copyright(C) 2014 Yuichi Kawamoto (ykm2006083001 AT gmail DOT com)
 All rights reserved.

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option)
 any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details.

 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

 The full GNU General Public License is included in this distribution in the
 file called LICENSE.
 */

#include <libindi/indiccd.h>
#include <qhyccd.h>

class QHYCCD : public INDI::CCD
{
public:
    QHYCCD();
    ~QHYCCD();

    void ISGetProperties(const char *dev);
	virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);

protected:
    // General device functions
    bool Connect();
    bool Disconnect();
    const char *getDefaultName();
    bool initProperties();
    bool updateProperties();

    // CCD specific functions
    bool StartExposure(float duration);
    bool AbortExposure();
    int SetTemperature(double temperature);
	virtual void addFITSKeywords(fitsfile *fptr, CCDChip *targetChip);
    void TimerHit();

private:

    // --- Utility functions ---

    float CalcTimeLeft();
    float CalcTimeSince(struct timeval *start);
    void  setupParams();
    void  grabImage();
	int   ResetTempControl(double targetTemp);

	// --- Private Properties ---

    // Are we exposing?
    bool InExposure;

    // Struct to keep timing
    struct timeval ExpStart;

	// duration to expose requested
    float ExposureRequest;

	// target temperature requested
    float TemperatureRequest;

    int   timerID;

	// minimum duration allowed
	float minDuration;

	// exposure aborted for primary chip
    bool AbortPrimaryFrame;

    // We declare the CCD temperature property
    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    // time the new value of temperature is set
    struct timeval TimeTemperatureControlStarted;

	// Temperature when the control is started
    double TemperatureWhenControlStarted;

    // camera handle for lzr's QHYCCD_Linux
    qhyccd_handle *CameraHandle;

	// user setting for temperature control ratio
	INumberVectorProperty *TemperatureControlRatioNV;
	INumber TemperatureControlRatioN[1];

};

#endif // INDI_QHYCCD_LINUX_H
