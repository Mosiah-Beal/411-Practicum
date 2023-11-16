#ifndef _WEATHERMONITOR_H_
#define _WEATHERMONITOR_H_

#include <SinricProDevice.h>
#include <Capabilities/TemperatureSensor.h>
#include <Capabilities/PushNotification.h>
#include <Capabilities/ToggleController.h>

class WeatherMonitor 
: public SinricProDevice
, public TemperatureSensor<WeatherMonitor>
, public PushNotification<WeatherMonitor>
, public ToggleController<WeatherMonitor> {
  friend class TemperatureSensor<WeatherMonitor>;
  friend class PushNotification<WeatherMonitor>;
  friend class ToggleController<WeatherMonitor>;
public:
  WeatherMonitor(const String &deviceId) : SinricProDevice(deviceId, "WeatherMonitor") {};
};

#endif
