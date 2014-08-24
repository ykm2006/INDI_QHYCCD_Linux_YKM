#include <sys/time.h>
#include <memory>

#include "indi_qhyccd_linux.h"

const int POLLMS           = 500;       /* Polling interval 500 ms */
const int MAX_CCD_TEMP     = 45;		/* Max CCD temperature */
const int MIN_CCD_TEMP	   = -55;		/* Min CCD temperature */
const float TEMP_THRESHOLD = .25;		/* Differential temperature threshold (C)*/

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value

std::auto_ptr<QHYCCD> _QHYCCD(0);

void ISInit()
{
    static int isInit =0;
    if (isInit == 1)
        return;

     isInit = 1;
     if(_QHYCCD.get() == 0) _QHYCCD.reset(new QHYCCD());
}

void ISGetProperties(const char *dev)
{
         ISInit();
         _QHYCCD->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
         ISInit();
         _QHYCCD->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
         ISInit();
         _QHYCCD->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
         ISInit();
         _QHYCCD->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (
		const char *dev,
		const char *name,
		int sizes[],
		int blobsizes[],
		char *blobs[],
		char *formats[],
		char *names[],
		int n)
{
   INDI_UNUSED(dev);
   INDI_UNUSED(name);
   INDI_UNUSED(sizes);
   INDI_UNUSED(blobsizes);
   INDI_UNUSED(blobs);
   INDI_UNUSED(formats);
   INDI_UNUSED(names);
   INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
     ISInit();
     _QHYCCD->ISSnoopDevice(root);
}


QHYCCD::QHYCCD()
{
    InExposure = false;
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool QHYCCD::Connect()
{
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
    IDMessage(getDeviceName(), "QHYCCD disconnected successfully!");
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * QHYCCD::getDefaultName()
{
    return "QHYCCD_Linux";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool QHYCCD::initProperties()
{
    // Must init parent properties first!
    INDI::CCD::initProperties();

    // We set the CCD capabilities
    Capability cap;

    cap.canAbort = true;
    cap.canBin   = true;
    cap.canSubFrame = true;
    cap.hasCooler = true;
    cap.hasGuideHead = false;
    cap.hasShutter = true;
    cap.hasST4Port = false;

    SetCapability(&cap);

    // Add Debug, Simulator, and Configuration controls
    addAuxControls();

    return true;

}

/**************************************************************************************
** INDI is asking us to submit list of properties for the device
***************************************************************************************/
void QHYCCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);

    // If we are _already_ connected, let's define our temperature property to the client now
    if (isConnected())
    {
        // Define our only property temperature
        defineNumber(&TemperatureNP);
    }

}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool QHYCCD::updateProperties()
{
    // Call parent update properties first
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD
        setupParams();

        // Start the timer
        SetTimer(POLLMS);
    }

    return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void QHYCCD::setupParams()
{
    // Our CCD is an 8 bit CCD, 1280x1024 resolution, with 5.4um square pixels.
    SetCCDParams(1280, 1024, 8, 5.4, 5.4);

    // Let's calculate how much memory we need for the primary CCD buffer
    int nbuf;
    nbuf=PrimaryCCD.getXRes()*PrimaryCCD.getYRes() * PrimaryCCD.getBPP()/8;
    nbuf+=512;                      //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);
}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool QHYCCD::StartExposure(float duration)
{
    ExposureRequest=duration;

    // Since we have only have one CCD with one chip, we set the exposure duration of the primary CCD
    PrimaryCCD.setExposureDuration(duration);

    gettimeofday(&ExpStart,NULL);

    InExposure=true;

    // We're done
    return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool QHYCCD::AbortExposure()
{
    InExposure = false;
    return true;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int QHYCCD::SetTemperature(double temperature)
{
    TemperatureRequest = temperature;

    // 1 means it will take a while to change the temperature
    return 1;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float QHYCCD::CalcTimeLeft()
{
    double timesince;
    double timeleft;
    struct timeval now;
    gettimeofday(&now,NULL);

    timesince=(double)(now.tv_sec * 1000.0 + now.tv_usec/1000) - (double)(ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec/1000);
    timesince=timesince/1000;

    timeleft=ExposureRequest-timesince;
    return timeleft;
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void QHYCCD::TimerHit()
{
    long timeleft;

    IDLog("TimerHit().\n");

    if(isConnected() == false)
        return;  //  No need to reset timer if we are not connected anymore

    if (InExposure)
    {
        timeleft=CalcTimeLeft();

        // Less than a 0.1 second away from exposure completion
        // This is an over simplified timing method, check CCDSimulator and QHYCCD for better timing checks
        if(timeleft < 0.1)
        {
          /* We're done exposing */
           IDMessage(getDeviceName(), "Exposure done, downloading image...");

          // Set exposure left to zero
          PrimaryCCD.setExposureLeft(0);

          // We're no longer exposing...
          InExposure = false;

          /* grab and save image */
          grabImage();

        }
        else
         // Just update time left in client
         PrimaryCCD.setExposureLeft(timeleft);

    }

    // TemperatureNP is defined in INDI::CCD
    switch (TemperatureNP.s)
    {
      case IPS_IDLE:
      case IPS_OK:
        break;

      case IPS_BUSY:
        /* If target temperature is higher, then increase current CCD temperature */
        if (currentCCDTemperature < TemperatureRequest)
           currentCCDTemperature++;
        /* If target temperature is lower, then decrese current CCD temperature */
        else if (currentCCDTemperature > TemperatureRequest)
          currentCCDTemperature--;
        /* If they're equal, stop updating */
        else
        {
          TemperatureNP.s = IPS_OK;
          IDSetNumber(&TemperatureNP, "Target temperature reached.");

          break;
        }

	IDLog("SetTempleture to [%f]\n", currentCCDTemperature);


        IDSetNumber(&TemperatureNP, NULL);

        break;

      case IPS_ALERT:
        break;
    }

    SetTimer(POLLMS);
    return;
}

/**************************************************************************************
** Create a random image and return it to client
***************************************************************************************/
void QHYCCD::grabImage()
{
   // Let's get a pointer to the frame buffer
   char * image = PrimaryCCD.getFrameBuffer();

   // Get width and height
   int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX() * PrimaryCCD.getBPP()/8;
   int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();

   // Fill buffer with random pattern
   for (int i=0; i < height ; i++)
     for (int j=0; j < width; j++)
         image[i*width+j] = rand() % 255;

   IDMessage(getDeviceName(), "Download complete.");

   // Let INDI::CCD know we're done filling the image buffer
   ExposureComplete(&PrimaryCCD);
}
