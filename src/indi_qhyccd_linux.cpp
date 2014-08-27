// TODO
// - accurate timeleft calculation 
//   DONE.  Stop is not supported yet.
// - temperature control
//   DONE.  PID control needs to be improved.
// - code cleanup
// - finetune PID control
// - configure temp control speed
// - xfer speed setting
// - guider ccd support
// - check for other models
// - make model name visible for client
// - comment header
// - stop support

#include <sys/time.h>
#include <memory>
#include <math.h>
#include <libindi/eventloop.h>

#include "indi_qhyccd_linux.h"

const int    POLLMS         = 1000;  /* Polling interval 500 ms */
const int    MAX_CCD_TEMP   = 45;    /* Max CCD temperature */
const int    MIN_CCD_TEMP   = -55;   /* Min CCD temperature */
const float  TEMP_THRESHOLD = .25;   /* Differential temperature threshold (C) */
const double DEFAULT_GAIN   = 14.0;
const double DEFAULT_OFFSET = 107.0;

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value

std::auto_ptr <QHYCCD> _QHYCCD(0);

/**************************************************************************************
** INDI Wrappers
***************************************************************************************/
void ISInit()
{
    static int isInit = 0;

    if (isInit == 1)
        return;

    isInit = 1;
    if (_QHYCCD.get() == 0)
        _QHYCCD.reset(new QHYCCD());
}

void ISGetProperties(const char *dev)
{
    ISInit();
    _QHYCCD->ISGetProperties(dev);
}

void
ISNewSwitch(const char *dev, const char *name, ISState * states,
            char *names[], int num)
{
    ISInit();
    _QHYCCD->ISNewSwitch(dev, name, states, names, num);
}

void
ISNewText(const char *dev, const char *name, char *texts[], char *names[],
          int num)
{
    ISInit();
    _QHYCCD->ISNewText(dev, name, texts, names, num);
}

void
ISNewNumber(const char *dev, const char *name, double values[],
            char *names[], int num)
{
    ISInit();
    _QHYCCD->ISNewNumber(dev, name, values, names, num);
}

void
ISNewBLOB(const char *dev,
          const char *name,
          int sizes[],
          int blobsizes[],
          char *blobs[], char *formats[], char *names[], int n)
{
    ISInit();
    _QHYCCD->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names,
                       n);
}

void ISSnoopDevice(XMLEle * root)
{
    ISInit();
    _QHYCCD->ISSnoopDevice(root);
}

/**************************************************************************************
** Construction
***************************************************************************************/
QHYCCD::QHYCCD()
{
    IDLog("%s():\n", __FUNCTION__);

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
    return "QHY CCD(QHYCCD_Linux Driver based)";
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

    // FLI_CCD sets user interface such as ports and reset switches here,
    // currently we have no such UIs

    // We set the CCD capabilities
    Capability cap;

    // currently QHYCCD_Linux does not support abort
    //cap.canAbort = true;
    cap.canAbort = false;
    cap.canBin = true;
    // currently QHYCCD_Linux does not support subframe
    //cap.canSubFrame = true;
    cap.canSubFrame = false;
    cap.hasCooler = true;
    cap.hasGuideHead = false;
    // currently QHYCCD_Linux does not support shutter
    //cap.hasShutter = true;
    cap.hasShutter = false;
    cap.hasST4Port = false;

    SetCapability(&cap);

    return true;

}

/**************************************************************************************
** INDI is asking us to submit list of properties for the device
***************************************************************************************/
void QHYCCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);

    // If we are _already_ connected, let's define our temperature property to the client now
    if (isConnected()) {
        // Define our only property temperature
        defineNumber(&TemperatureNP);
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

        setupParams();

        // Start the timer
        timerID = SetTimer(POLLMS);
    } else {
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

    char id[0x20] = { 0 };
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

        if (strncmp(id, "IC8300", 6) == 0) {
            found = true;
            break;
        }
    }

    if (!found) {
        IDLog("no cameras found.\n");
        IDMessage(getDeviceName(), "  no cameras found.\n");
        ReleaseQHYCCDResource();
        return false;
    }

    IDLog("found camera [%s]\n", id);
    IDMessage(getDeviceName(), "  found camera [%s]\n", id);

    hCamera = OpenQHYCCD(id);
    if (!hCamera) {
        IDLog("Could not open camera with id [%s]\n", id);
        IDMessage(getDeviceName(),
                  "  Could not open camera with id [%s]\n", id);
        ReleaseQHYCCDResource();
        return false;
    }

    IDLog("camera [%s] open successful.", id);
    IDMessage(getDeviceName(), "  camera [%s] open successful.", id);

    ret = InitQHYCCD(hCamera);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not initialize camera [%d]\n", ret);
        IDMessage(getDeviceName(), "  Could not initialize camera [%d]",
                  ret);
        CloseQHYCCD(hCamera);
        ReleaseQHYCCDResource();
        return false;
    }

    // Try to set gain.  Gain and Offset needs to be set from Client later

    double gainMin, gainMax, gainStep;
    ret =
        GetQHYCCDParamMinMaxStep(hCamera, CONTROL_GAIN, &gainMin, &gainMax,
                                 &gainStep);

    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for GAIN (%d)\n", ret);
    } else {
        IDLog("Gain settings (%.1f, %.1f, +%.1f), setting to %.1f\n",
              gainMin, gainMax, gainStep, DEFAULT_GAIN);
        ret = SetQHYCCDParam(hCamera, CONTROL_GAIN, DEFAULT_GAIN);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set gain to %.1f (%d)!\n", DEFAULT_GAIN, ret);
        }
    }

    // Try to set offset
    ret =
        GetQHYCCDParamMinMaxStep(hCamera, CONTROL_OFFSET, &gainMin,
                                 &gainMax, &gainStep);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for OFFSET (%d)\n", ret);
    } else {
        IDLog("Offset settings (%.1f, %.1f, +%.1f), setting to %.1f\n",
              gainMin, gainMax, gainStep, DEFAULT_OFFSET);
        ret = SetQHYCCDParam(hCamera, CONTROL_OFFSET, DEFAULT_OFFSET);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set offset to 107.0 (%d)!\n", ret);
        }
    }

    // control speed.  needs to be set from the client later.
    SetQHYCCDParam(hCamera, CONTROL_SPEED, 0);

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

    CloseQHYCCD(hCamera);
    hCamera = NULL;
    ReleaseQHYCCDResource();

    IDMessage(getDeviceName(), "QHYCCD disconnected successfully!");
    return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void QHYCCD::setupParams()
{
    IDLog("%s\n", __FUNCTION__);

    double chipw, chiph;
    int imagew, imageh;
    double pixelw, pixelh;
    int bpp;

    GetQHYCCDChipInfo(hCamera, &chipw, &chiph, &imagew, &imageh, &pixelw,
                      &pixelh, &bpp);
    SetCCDParams(imagew, imageh, bpp, pixelw, pixelh);

    IDLog("w[%d], h[%d], bpp[%d], pw[%.1f], ph[%.1f]\n", 
          imagew, imageh, bpp, pixelw, pixelh);

    int nbuf = GetQHYCCDMemLength(hCamera);

    IDLog("nbuf = [%d]\n", nbuf);

    PrimaryCCD.setFrameBufferSize(nbuf + 512, true);    // give some extra ends

    // Get Temperature
    // reconsider initialization timing, according to FLI example.
    double temp = GetQHYCCDParam(hCamera, CONTROL_CURTEMP);

    IDMessage(getDeviceName(), "The CCD Temperature is %f.\n", temp);

    currentCCDTemperature = temp;
    TemperatureN[0].min = MIN_CCD_TEMP;
    TemperatureN[0].max = MAX_CCD_TEMP;
    IUUpdateMinMax(&TemperatureNP);
    IDSetNumber(&TemperatureNP, NULL);

    TempStart = temp;
    TemperatureNP.s = IPS_BUSY;
    IDSetNumber(&TemperatureNP, NULL);

    // set the time the new value of temperature is set
    gettimeofday(&TimeTempStart, NULL);

    // binning
    SetQHYCCDBinMode(hCamera, PrimaryCCD.getBinX(), PrimaryCCD.getBinY());

    minDuration = 0.05;

    return;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int QHYCCD::SetTemperature(double temperature)
{
    IDLog("%s(%f):\n", __FUNCTION__, temperature);
    TemperatureRequest = temperature;
    TempStart = GetQHYCCDParam(hCamera, CONTROL_CURTEMP);

    // set the time the new value of temperature is set
    gettimeofday(&TimeTempStart, NULL);

    if(fabs(TempStart - temperature) <= TEMP_THRESHOLD) {
        // within threshold.   temp control completed.
        return 1;
    } else {
        // 0 means it will take a while to change the temperature
        return 0;
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
        //SetQHYCCDShutter(hCamera, SHUTTER_CLOSED);
    } else {
        IDLog("Setting Shutter Closed. Not supported currently.\n");
        //SetQHYCCDShutter(hCamera, SHUTTER_CLOSED);
    }

    if(durationControl) {
        duration = minDuration;
    }

    int ret = SetQHYCCDParam(hCamera, CONTROL_EXPOSURE,
                             (int) (duration * 1000 * 1000));
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not set exposure time to %d us\n",
              (int) (duration * 1000 * 1000));
        return false;
    }

    IDLog("Set exposure time.. begin exposing.\n");

    ret = ExpQHYCCDSingleFrame(hCamera);
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
    double timesince;
    double timeleft;
    struct timeval now;
    gettimeofday(&now, NULL);

    timesince =
        (double) (now.tv_sec * 1000.0 + now.tv_usec / 1000) -
        (double) (ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec / 1000);
    timesince = timesince / 1000;

    timeleft = ExposureRequest - timesince;
    return timeleft;
}

float QHYCCD::CalcTimeSince(struct timeval *start)
{
    double timesince;
    struct timeval now;

    gettimeofday(&now, NULL);

    timesince = (double) (now.tv_sec * 1000.0 + now.tv_usec / 1000) -
        (double) (start->tv_sec * 1000.0 + start->tv_usec / 1000);
    timesince = timesince / 1000;

    return timesince;
}

void QHYCCD::addFITSKeywords(fitsfile *fptr, CCDChip *targetChip)
{
    INDI::CCD::addFITSKeywords(fptr, targetChip);

    int status=0;
    fits_update_key_s(fptr, TDOUBLE, "CCD-TEMP", &(TemperatureN[0].value), "CCD Temperature (Celcius)", &status);
    fits_write_date(fptr, &status);
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void QHYCCD::TimerHit()
{
    IDLog("%s():\n", __FUNCTION__);

    // nexttimer is variable interval time in ms
    int nexttimer = POLLMS;

    if (isConnected() == false)
        return;                 //  No need to reset timer if we are not connected anymore

    if (InExposure) {
        if (AbortPrimaryFrame) {
            // if exposure is aborted, we need to send the camera StopExposure command.  Not yet supported.
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
    // TemperatureNP is defined in INDI::CCD
    float timesince = CalcTimeSince(&TimeTempStart);
    float targettemp;

    switch (TemperatureNP.s) {
    case IPS_IDLE:
    case IPS_OK:
        if (fabs(currentCCDTemperature - TemperatureRequest) > TEMP_THRESHOLD) {

            IDLog("over threshold, start controlling.\n");

            TemperatureNP.s = IPS_BUSY;
            IDSetNumber(&TemperatureNP, NULL);

            gettimeofday(&TimeTempStart, NULL);

            break;
        }
        break;

    case IPS_BUSY:
    {
        currentCCDTemperature = GetQHYCCDParam(hCamera, CONTROL_CURTEMP);

//        IDLog("Current temp: %.1f   Target temp: %.1f\n", currentCCDTemperature, TemperatureRequest);

        /* If target temperature is higher, then increase current CCD temperature */
        if (currentCCDTemperature < TemperatureRequest) {

            targettemp = TempStart + 10.0f / 180 * timesince;
            if (targettemp > TemperatureRequest) {
                targettemp = TemperatureRequest;
            }

            /* If target temperature is lower, then decrese current CCD temperature */
        } else if (currentCCDTemperature > TemperatureRequest) {
            targettemp = TempStart - 10.0f / 180 * timesince;
            if (targettemp < TemperatureRequest) {
                targettemp = TemperatureRequest;
            }

        }

//        IDLog("timesince:[%f], targettemp=[%f]\n", timesince, targettemp);

        ControlQHYCCDTemp(hCamera, targettemp);

        // read current PWM for monitor

//        double pwm = GetQHYCCDParam(hCamera, CONTROL_CURPWM);
//        IDLog("current pwm=[%f]\n", pwm);

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

    //IDLog("nexttimer=[%d]\n", nexttimer);
    SetTimer(nexttimer);
    return;
}

void QHYCCD::grabImage()
{
    // Let's get a pointer to the frame buffer
    unsigned char *image = (unsigned char *) PrimaryCCD.getFrameBuffer();

    int h = 0, w = 0, bpp = 0, ch = 0;

    int ret = GetQHYCCDSingleFrame(hCamera, &w, &h, &bpp, &ch, image);

    if (ret >= 0) {
        IDLog("Done exposing.\n");
        IDLog("w=[%d], h=[%d], bpp=[%d], ch=[%d]\n", w, h, bpp, ch);
        IDMessage(getDeviceName(), "Download complete.");
        // Let INDI::CCD know we're done filling the image buffer
        ExposureComplete(&PrimaryCCD);
    }
    return;
}
