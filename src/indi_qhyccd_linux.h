#ifndef INDI_QHYCCD_LINUX_H
#define INDI_QHYCCD_LINUX_H

#include <libindi/indiccd.h>
#include <qhyccd.h>

class QHYCCD : public INDI::CCD
{
public:
    QHYCCD();
    ~QHYCCD();

    void ISGetProperties(const char *dev);

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
    // Utility functions
    float CalcTimeLeft();
    float CalcTimeSince(struct timeval *start);
    void  setupParams();
    void  grabImage();

    // Are we exposing?
    bool InExposure;
    // Struct to keep timing
    struct timeval ExpStart;

    float ExposureRequest;
    float TemperatureRequest;
    int   timerID;
	float minDuration;

    bool AbortPrimaryFrame;

    // We declare the CCD temperature property
    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    // time the new value of temperature is set
    struct timeval TimeTempStart;
    double TempStart;

    // QHYCCD_Linux
    qhyccd_handle *hCamera;

};

#endif // INDI_QHYCCD_LINUX_H
