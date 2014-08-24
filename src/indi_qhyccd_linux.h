#ifndef INDI_QHYCCD_LINUX_H
#define INDI_QHYCCD_LINUX_H

#include <libindi/indiccd.h>

class QHYCCD : public INDI::CCD
{
public:
    QHYCCD();

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
    void TimerHit();

private:
    // Utility functions
    float CalcTimeLeft();
    void  setupParams();
    void  grabImage();

    // Are we exposing?
    bool InExposure;
    // Struct to keep timing
    struct timeval ExpStart;

    float ExposureRequest;
    float TemperatureRequest;
    int   timerID;

    // We declare the CCD temperature property
    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

};

#endif // INDI_QHYCCD_LINUX_H
