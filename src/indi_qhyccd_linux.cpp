/*
 QHYCCD INDI Driver based on QHYCCD_Linux

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

/* ------------------------------------------------------------------------------------
   TODO
   - accurate timeleft calculation 
     DONE.  Stop is not supported yet.

   - temperature control
     DONE.  PID control needs to be improved.
  
   - code cleanup
     (refs #3276)
  
   - finetune PID control
     not done.   Will depend on lzr's code.
     Got to be not not too bad with 1 second interval

   - configure temp control speed

   - guider ccd support

   - check for other models
 
   - make model name visible for client

   - comment header

   - stop support
   --------------------------------------------------------------------------------- */

#include <sys/time.h>
#include <memory>
#include <math.h>
#include <libindi/eventloop.h>

#include "indi_qhyccd_linux.h"

const int    POLLMS         = 1000;  /* Polling interval 500 ms */
const int    MAX_CCD_TEMP   = 45;    /* Max CCD temperature */
const int    MIN_CCD_TEMP   = -55;   /* Min CCD temperature */
const int    MAX_DEVICES    = 20;    /* Max devices we can open */
const float  TEMP_THRESHOLD = .25;   /* Differential temperature threshold (C) */
const double DEFAULT_GAIN   = 14.0;
const double DEFAULT_OFFSET = 107.0;

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value

static int cameraCount;
QHYCCD *cameras[MAX_DEVICES];

/**************************************************************************************
** INDI Wrappers
***************************************************************************************/
static void cleanup() {
    for(int i=0; i<cameraCount; i++) {
        delete cameras[i];
    }
}

void ISInit()
{
    static bool isInit = false;

    if(isInit) {
        return;
    }

    // let's just create one camera for now.
    // multiple device support
    int ret = InitQHYCCDResource();

    if (ret == QHYCCD_SUCCESS) {

        IDLog("InitQHYCCDResource() success.\n");
        
        // lets count number of cameras
        cameraCount = ScanQHYCCD();
        if (cameraCount > 0) {

            IDLog("found [%d] QHYCCD Cameras.\n", cameraCount);

            char id[0x20] = {0};
            bool failed = false;

            for (int i = 0; i < cameraCount; i++) {
                cameras[i] = new QHYCCD();
                QHYCCD* camera = cameras[i];

                ret = GetQHYCCDId(i, id);
                if (ret == QHYCCD_SUCCESS) {
                    IDLog("Found camera with id [%s]\n", id);

                    id[strlen(id) - 17] = '\0';

                    camera->SetName(id);

                } else {
                    IDLog("Could not get CCD id for index[%d]\n", i);
                    failed = true;
                    break;
                }
            }
            if(!failed) {
                ReleaseQHYCCDResource();
                atexit(cleanup);
                isInit = true;
                return;
            }
        } else {
            IDLog("Could not find a device [%d]\n", cameraCount);
            ReleaseQHYCCDResource();
        }
    } else {
        IDLog("InitQHYCCDResource() failed.\n");
    }
    return;
}

void ISGetProperties(const char *dev)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        if(dev==NULL || !strcmp(dev, camera->GetName())) {
            camera->ISGetProperties(dev);
            if(dev != NULL) {
                break;
            }
        }
    }
}

void
ISNewSwitch(const char *dev, const char *name, ISState * states,
            char *names[], int num)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        if(dev==NULL || !strcmp(dev, camera->GetName())) {
            camera->ISNewSwitch(dev, name, states, names, num);
            if(dev != NULL) {
                break;
            }
        }
    }
}

void
ISNewText(const char *dev, const char *name, char *texts[], char *names[],
          int num)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        if(dev==NULL || !strcmp(dev, camera->GetName())) {
            camera->ISNewText(dev, name, texts, names, num);
            if(dev != NULL) {
                break;
            }
        }
    }
}

void
ISNewNumber(const char *dev, const char *name, double values[],
            char *names[], int num)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        if(dev==NULL || !strcmp(dev, camera->GetName())) {
            camera->ISNewNumber(dev, name, values, names, num);
            if(dev != NULL) {
                break;
            }
        }
    }
}

void
ISNewBLOB(const char *dev,
          const char *name,
          int sizes[],
          int blobsizes[],
          char *blobs[], char *formats[], char *names[], int n)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        if(dev==NULL || !strcmp(dev, camera->GetName())) {
            camera->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
            if(dev != NULL) {
                break;
            }
        }
    }
}

void ISSnoopDevice(XMLEle * root)
{
    ISInit();

    for(int i=0; i<cameraCount; i++) {
        QHYCCD *camera = cameras[i];
        camera->ISSnoopDevice(root);
    }
}

/**************************************************************************************
** Construction
***************************************************************************************/
QHYCCD::QHYCCD()
{
    IDLog("%s():\n", __FUNCTION__);

    // for now, let's set name to default name.
    if(*getDeviceName() == '\0') {
        strncpy(name, getDefaultName(), MAXINDINAME);
    } else {
        strncpy(name, getDeviceName(), MAXINDINAME);
    }

    setVersion(0, 1);
}

QHYCCD::~QHYCCD()
{
    IDLog("%s()\n", __FUNCTION__);
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *QHYCCD::getDefaultName()
{
    IDLog("%s():\n", __FUNCTION__);

    return GetName();
//    return "QHYCCD(multi device test)";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool QHYCCD::initProperties()
{
    IDLog("%s():\n", __FUNCTION__);

    // Must init parent properties first!
    INDI::CCD::initProperties();

    InExposure = false;
    AbortPrimaryFrame = false;

    // initialize the UI properties
    if(strncmp(GetName(), "QHY5", 4) != 0) {
        IDLog("cool camera.\n");
        IUFillNumber(&TemperatureControlRatioN[0], "TMP_CTRL_RATIO",
                     "Ctrl Ratio(degree/180sec)","%2.2f", 1.0f, 50.0f, 0.0f, 10.0f);
        IUFillNumberVector(&TemperatureControlRatioNP, TemperatureControlRatioN, 1, getDeviceName(),
                           "TEMPERATURE_CONTROL_RATIO","Temp Ctrl",
                           "Main Control",IP_RW,60,IPS_IDLE);
    } else {
        IDLog("non-cool camera.\n");
    }

    return true;

}

/**************************************************************************************
** INDI is asking us to submit list of properties for the device
***************************************************************************************/
void QHYCCD::ISGetProperties(const char *dev)
{
    IDLog("%s(%s)\n", __FUNCTION__, dev);

    INDI::CCD::ISGetProperties(dev);

    // If we are _already_ connected, let's define our temperature property to the client now
    if (isConnected()) {

        defineNumber(&TemperatureNP);
        defineNumber(&TemperatureControlRatioNP);
    }

    // Add Debug, Simulator, and Configuration controls
    addAuxControls();
}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool QHYCCD::updateProperties()
{
    // Call parent update properties first
    INDI::CCD::updateProperties();

    if (isConnected()) {

        // Let's get parameters now from CCD
        defineNumber(&TemperatureNP);
        defineNumber(&TemperatureControlRatioNP);

        setupParams();

        // Start the timer
        timerID = SetTimer(POLLMS);
    } else {
        deleteProperty(TemperatureNP.name);
        deleteProperty(TemperatureControlRatioNP.name);
        rmTimer(timerID);
    }

    return true;
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool QHYCCD::Connect()
{
    IDLog("%s()\n", __FUNCTION__);
    
    int ret = InitQHYCCDResource();
    if (ret != QHYCCD_SUCCESS) {
        IDLog("InitQHYCCDResource() failed.\n");
        return false;
    } else {
        IDLog("InitQHYCCDResource() success.\n");
    }

    // lets count number of cameras
    int nCameras = ScanQHYCCD();
    if (nCameras < 0) {
        IDLog("Could not find a device [%d]\n", nCameras);
        ReleaseQHYCCDResource();
        return false;
    }

    IDMessage(getDeviceName(), "  found [%d] QHYCCD Cameras.\n", nCameras);

    char id[0x20]            = {0};
    char idPrimaryCCD[0x20]  = {0};
    bool found = false;

    for (int i = 0; i < nCameras; i++) {
        ret = GetQHYCCDId(i, id);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not get CCD id for index[%d]\n", i);
            IDMessage(getDeviceName(),
                      "  Could not get CCD id for index[%d]\n", i);
            continue;
        }
        IDLog("Found camera with id [%s]\n", id);
        IDMessage(getDeviceName(), "  Found camera with id [%s]\n", id);

        if (strncmp(id, GetName(), strlen(GetName())) == 0) {
            IDLog("Found Primary CCD [%s]\n", id);
            strcpy(idPrimaryCCD, id);
            found = true;
        }
    }

    // initialize Primary CCD

    if (!found) {
        IDLog("no cameras found.\n");
        IDMessage(getDeviceName(), "  no cameras found.\n");
        ReleaseQHYCCDResource();
        return false;
    }

    IDLog("found primary camera [%s]\n", idPrimaryCCD);
    IDMessage(getDeviceName(), "  found primary camera [%s]\n", idPrimaryCCD);

    HandlePrimaryCCD = OpenQHYCCD(idPrimaryCCD);
    if (!HandlePrimaryCCD) {
        IDLog("Could not open camera with id [%s]\n", idPrimaryCCD);
        IDMessage(getDeviceName(),
                  "  Could not open camera with id [%s]\n", idPrimaryCCD);
        ReleaseQHYCCDResource();
        return false;
    }

    IDLog("primary camera [%s] open successful.\n", idPrimaryCCD);
    IDMessage(getDeviceName(), "  primary camera [%s] open successful.", idPrimaryCCD);

    ret = InitQHYCCD(HandlePrimaryCCD);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not initialize camera [%d]\n", ret);
        IDMessage(getDeviceName(), "  Could not initialize camera [%d]",
                  ret);
        CloseQHYCCD(HandlePrimaryCCD);
        ReleaseQHYCCDResource();
        return false;
    }

    // Try to set gain.  Gain and Offset needs to be set from Client later

    double gainMin, gainMax, gainStep;
    ret =
        GetQHYCCDParamMinMaxStep(HandlePrimaryCCD, CONTROL_GAIN, &gainMin, &gainMax,
                                 &gainStep);

    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for GAIN (%d)\n", ret);
    } else {
        IDLog("Gain settings (%.1f, %.1f, +%.1f), setting to %.1f\n",
              gainMin, gainMax, gainStep, DEFAULT_GAIN);
        ret = SetQHYCCDParam(HandlePrimaryCCD, CONTROL_GAIN, DEFAULT_GAIN);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set gain to %.1f (%d)!\n", DEFAULT_GAIN, ret);
        }
    }

    // Try to set offset
    ret =
        GetQHYCCDParamMinMaxStep(HandlePrimaryCCD, CONTROL_OFFSET, &gainMin,
                                 &gainMax, &gainStep);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for OFFSET (%d)\n", ret);
    } else {
        IDLog("Offset settings (%.1f, %.1f, +%.1f), setting to %.1f\n",
              gainMin, gainMax, gainStep, DEFAULT_OFFSET);
        ret = SetQHYCCDParam(HandlePrimaryCCD, CONTROL_OFFSET, DEFAULT_OFFSET);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set offset to 107.0 (%d)!\n", ret);
        }
    }

    // We set the CCD capabilities.
    Capability cap;

    // currently QHYCCD_Linux does not support abort
    //cap.canAbort = true;
    cap.canAbort = false;
    cap.canBin = true;
    // currently QHYCCD_Linux does not support subframe
    //cap.canSubFrame = true;
    cap.canSubFrame = false;

    if(IsQHYCCDCool(HandlePrimaryCCD)) {
        cap.hasCooler = true;
    } else {
        cap.hasCooler = false;
    }

    // currently all QHYCCD does not have guider head
    cap.hasGuideHead = false;

    // currently QHYCCD_Linux does not support shutter
    //cap.hasShutter = true;
    cap.hasShutter = false;

    // QHYCCD_Linux does not have detection for guider port
    // then lets assume only QHY5 series has the port
    if(strncpy(GetName(), "QHY5", 4) == 0) {
        cap.hasST4Port = true;
    } else {
        cap.hasST4Port = false;
    }

    SetCapability(&cap);

    // control speed.  needs to be set from the client later.
    SetQHYCCDParam(HandlePrimaryCCD, CONTROL_SPEED, 0);

    IDLog("Camera initialized successfully.\n");
    IDMessage(getDeviceName(), "  Camera initialized successfully.");

    IDMessage(getDeviceName(), "QHYCCD connected successfully!");

    // Let's set a timer that checks teleCCDs status every POLLMS milliseconds.
    SetTimer(POLLMS);

    return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool QHYCCD::Disconnect()
{
    IDLog("%s()\n", __FUNCTION__);

    CloseQHYCCD(HandlePrimaryCCD);

    HandlePrimaryCCD = NULL;

    ReleaseQHYCCDResource();

    IDMessage(getDeviceName(), "QHYCCD disconnected successfully!");
    return true;
}

/**************************************************************************************
** Setting up CCD parameters
** basically getting camera info and set parameters in driver
***************************************************************************************/
void QHYCCD::setupParams()
{
    IDLog("%s\n", __FUNCTION__);

    double chipw, chiph;
    int imagew, imageh;
    double pixelw, pixelh;
    int bpp;

    GetQHYCCDChipInfo(HandlePrimaryCCD, &chipw, &chiph, &imagew, &imageh, &pixelw,
                      &pixelh, &bpp);
    SetCCDParams(imagew, imageh, bpp, pixelw, pixelh);

    IDLog("prime: w[%d], h[%d], bpp[%d], pw[%.1f], ph[%.1f]\n", 
          imagew, imageh, bpp, pixelw, pixelh);

    int nbuf = GetQHYCCDMemLength(HandlePrimaryCCD);

    IDLog("nbuf = [%d]\n", nbuf);

    PrimaryCCD.setFrameBufferSize(nbuf + 512, true);    // give some extra ends

    if(IsQHYCCDCool(HandlePrimaryCCD)) {
        ResetTempControl(TemperatureRequest);

        TemperatureN[0].value = GetQHYCCDParam(HandlePrimaryCCD, CONTROL_CURTEMP);
        TemperatureN[0].min = MIN_CCD_TEMP;
        TemperatureN[0].max = MAX_CCD_TEMP;
        IUUpdateMinMax(&TemperatureNP);
        IDSetNumber(&TemperatureNP, NULL);

        TemperatureNP.s = IPS_BUSY;
        IDSetNumber(&TemperatureNP, NULL);

    }
    // binning
    SetQHYCCDBinMode(HandlePrimaryCCD, PrimaryCCD.getBinX(), PrimaryCCD.getBinY());

    minDuration = 0.05;

    return;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int QHYCCD::SetTemperature(double temperature)
{
    if(IsQHYCCDCool(HandlePrimaryCCD)) {
        IDLog("%s(%f):\n", __FUNCTION__, temperature);
        TemperatureRequest = temperature;
        return ResetTempControl(temperature);
    } else {
        return 1;
    }
}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool QHYCCD::StartExposure(float duration)
{
    IDLog("%s(%f)\n", __FUNCTION__, duration);

    bool shutterOpen, durationControl;

    //    LIGHT_FRAME=0, BIAS_FRAME, DARK_FRAME, FLAT_FRAME
    switch(PrimaryCCD.getFrameType()) {
    case CCDChip::LIGHT_FRAME:
        // shutter open, duration specified
        shutterOpen = true;
        durationControl = false;
        break;
    case CCDChip::BIAS_FRAME:
        // shutter closed, duration is minimal
        shutterOpen = false;
        durationControl = true;
        break;
    case CCDChip::DARK_FRAME:
        // shutter closed, duration specified
        shutterOpen = false;
        durationControl = false;
        break;
    case CCDChip::FLAT_FRAME:
        // shutter open, duration minimal
        shutterOpen = true;
        durationControl = true;
        break;
    default:
        IDLog("unknown value set for FrameType.[%d]\n", PrimaryCCD.getFrameType());
    }

    if(shutterOpen) {
        IDLog("Setting Shutter Open. Not supported currently.\n");
        //SetQHYCCDShutter(HandlePrimaryCCD, SHUTTER_OPEN);
    } else {
        IDLog("Setting Shutter Closed. Not supported currently.\n");
        //SetQHYCCDShutter(HandlePrimaryCCD, SHUTTER_CLOSED);
    }

    if(durationControl) {
        IDLog("Duration is controlled to minimal exposure.\n");
        duration = minDuration;
    }

    // QHYCCD_Linux exposure is microsecond
    int ret = SetQHYCCDParam(HandlePrimaryCCD, CONTROL_EXPOSURE,
                             (int) (duration * 1000 * 1000));
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not set exposure time to %d us\n",
              (int) (duration * 1000 * 1000));
        return false;
    }

    IDLog("Set exposure time.. begin exposing.\n");

    ret = ExpQHYCCDSingleFrame(HandlePrimaryCCD);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not start exposure [%d]\n", ret);
        return false;
    }

    ExposureRequest = duration;

    // Since we have only one CCD with one chip, we set the exposure duration of the primary CCD
    PrimaryCCD.setExposureDuration(duration);

    gettimeofday(&ExpStart, NULL);

    InExposure = true;
    AbortPrimaryFrame = false;

    // We're done
    IDLog("Started exposing.\n");

    return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool QHYCCD::AbortExposure()
{
    if (!InExposure) {
        return true;
    }

    AbortPrimaryFrame = true;
    IDMessage(getDeviceName(), "Abort Selected, but current QHYCCD_Linux does not support Abort.\n");

    return true;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float QHYCCD::CalcTimeLeft()
{
    double timeSince;
    double timeLeft;
    struct timeval now;
    gettimeofday(&now, NULL);

    timeSince = CalcTimeSince(&ExpStart);
    timeLeft = ExposureRequest - timeSince;

    return timeLeft;
}

float QHYCCD::CalcTimeSince(struct timeval *start)
{
    double timeSince;
    struct timeval now;

    gettimeofday(&now, NULL);

    timeSince = (double) (now.tv_sec * 1000.0 + now.tv_usec / 1000) -
        (double) (start->tv_sec * 1000.0 + start->tv_usec / 1000);
    timeSince = timeSince / 1000;

    return timeSince;
}

/**************************************************************************************
 * got from fli_ccd.cpp in indi_3rdparty library
***************************************************************************************/
void QHYCCD::addFITSKeywords(fitsfile *fptr, CCDChip *targetChip)
{
    INDI::CCD::addFITSKeywords(fptr, targetChip);

    int status=0;
    fits_update_key_s(fptr, TDOUBLE, "CCD-TEMP", &(TemperatureN[0].value), "CCD Temperature (Celcius)", &status);

// does not work, not sure why
//    fits_write_date(fptr, &status);
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void QHYCCD::TimerHit()
{
    //IDLog("%s():\n", __FUNCTION__);

    // nexttimer is variable interval time in ms
    int nexttimer = POLLMS;

    if (isConnected() == false)
        return;                 //  No need to reset timer if we are not connected anymore

    if (InExposure) {
        if (AbortPrimaryFrame) {
            // if exposure is aborted, we need to send the camera StopExposure command.
            // Not yet supported by lzr's linux driver,
            // so actually nothing changed.
            // InExposure = false;
            AbortPrimaryFrame = false;
        } else {
            float timeleft;
            timeleft = CalcTimeLeft();

            IDLog("CCD Exposure left: %g - Requset: %g\n", timeleft,
                  ExposureRequest);

            if (timeleft < 0.0) {
                timeleft = 0.0;
            }

            PrimaryCCD.setExposureLeft(timeleft);

            if (timeleft * 1000.0 < (float) POLLMS) {
                //IDLog("timeleft[%f] < POLLMS[%f]\n", timeleft * 1000.0, (float)POLLMS);
                if (timeleft <= 0.001) {
                    /* We're done exposing */
                    IDMessage(getDeviceName(), "Exposure done, downloading image...");
                    InExposure = false;
                    grabImage();
                    IDMessage(getDeviceName(), "download completed.");
                } else {
                    // we finetune next timer event
                    nexttimer = timeleft * 1000.0;
                }
            }
        }
    }

    if(IsQHYCCDCool(HandlePrimaryCCD)) {
        // TemperatureNP is defined in INDI::CCD
        float timesince = CalcTimeSince(&TimeTemperatureControlStarted);
        float targettemp;

        currentCCDTemperature = GetQHYCCDParam(HandlePrimaryCCD, CONTROL_CURTEMP);
        IDLog("Current temp: %.1f   Target temp: %.1f\n", currentCCDTemperature, TemperatureRequest);
        
        switch (TemperatureNP.s) {
        case IPS_IDLE:
        case IPS_OK:
            if (fabs(currentCCDTemperature - TemperatureRequest) > TEMP_THRESHOLD) {
                
                IDLog("kicked the threshold, start controlling.\n");

                // begin control
                TemperatureNP.s = IPS_BUSY;
                IDSetNumber(&TemperatureNP, NULL);

                ResetTempControl(TemperatureRequest);

                break;
            }
            break;

        case IPS_BUSY:
        {

            if (currentCCDTemperature < TemperatureRequest) {
                /* If target temperature is higher, then increase current CCD temperature */

                targettemp = TemperatureWhenControlStarted + TemperatureControlRatioN[0].value / 180 * timesince;
                if (targettemp > TemperatureRequest) {
                    targettemp = TemperatureRequest;
                }

            } else if (currentCCDTemperature > TemperatureRequest) {
                /* If target temperature is lower, then decrease current CCD temperature */

                targettemp = TemperatureWhenControlStarted - TemperatureControlRatioN[0].value / 180 * timesince;
                if (targettemp < TemperatureRequest) {
                    targettemp = TemperatureRequest;
                }

            }

            IDLog("timesince:[%.1f], targettemp=[%.1f]\n", timesince, targettemp);

            ControlQHYCCDTemp(HandlePrimaryCCD, targettemp);
            
            // read current PWM for monitor

            double pwm = GetQHYCCDParam(HandlePrimaryCCD, CONTROL_CURPWM);
            IDLog("current pwm=[%f]\n", pwm);

            if (fabs(currentCCDTemperature - TemperatureRequest) <= TEMP_THRESHOLD) {
                
                IDLog("reached at target temperature, go to IDLE state\n");

                TemperatureNP.s = IPS_OK;
            }

            IDSetNumber(&TemperatureNP, NULL);
            break;
        }

        case IPS_ALERT:
            break;
        default:
            break;
        }
    }

    //IDLog("nexttimer=[%d]\n", nexttimer);
    SetTimer(nexttimer);
    return;
}

/**************************************************************************************
  ResetTempControl(): reset the control and set the beginning of the control
  by reading current temperature of CCD and resetting the time keeper
***************************************************************************************/
int QHYCCD::ResetTempControl(double targetTemp)
{
    double temp = GetQHYCCDParam(HandlePrimaryCCD, CONTROL_CURTEMP);

    TemperatureRequest = targetTemp;
    currentCCDTemperature = temp;
    TemperatureWhenControlStarted = temp;

    // set the time the new value of temperature is set
    gettimeofday(&TimeTemperatureControlStarted, NULL);

    if(fabs(TemperatureWhenControlStarted - targetTemp) <= TEMP_THRESHOLD) {
        // within threshold.   temp control completed.
        return 1;
    } else {
        // 0 means it will take a while to change the temperature
        return 0;
    }
}

/**************************************************************************************
  grabImage(): read image data from chip and burn it to buffer
***************************************************************************************/
void QHYCCD::grabImage()
{
    // Let's get a pointer to the frame buffer
    unsigned char *image = (unsigned char *) PrimaryCCD.getFrameBuffer();

    int h = 0, w = 0, bpp = 0, ch = 0;

    int ret = GetQHYCCDSingleFrame(HandlePrimaryCCD, &w, &h, &bpp, &ch, image);

    if (ret >= 0) {
        IDLog("Done exposing.\n");
        IDLog("w=[%d], h=[%d], bpp=[%d], ch=[%d]\n", w, h, bpp, ch);
        IDMessage(getDeviceName(), "Download complete.");
        // Let INDI::CCD know we're done filling the image buffer
        ExposureComplete(&PrimaryCCD);
    }
    return;
}

bool QHYCCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device
    //IDLog("INDI::CCD::ISNewNumber %s\n",name);
    if(strcmp(dev,getDeviceName())==0)
    {
        //  This is for our device
        //  Now lets see if it's something we process here

        //IDLog("CCDSim::ISNewNumber %s\n",name);
        if(strcmp(name,"TEMPERATURE_CONTROL_RATIO")==0)
        {
            IUUpdateNumber(&TemperatureControlRatioNP, values, names, n);
            TemperatureControlRatioNP.s=IPS_OK;

            //  Reset our parameters now
            setupParams();
            IDSetNumber(&TemperatureControlRatioNP, NULL);
            //saveConfig();

            //IDLog("Frame set to %4.0f,%4.0f %4.0f x %4.0f\n",CcdFrameN[0].value,CcdFrameN[1].value,CcdFrameN[2].value,CcdFrameN[3].value);
            //seeing=SimulatorSettingsN[0].value;
            return true;
        }
    }
    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

