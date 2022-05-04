
#include "rollingAverage.h"
#include <time.h>


// bigger coefficient means faster
RollingAverage::RollingAverage(float _initial, float _downCoefficient, float _upCoefficient)
{
  value = _initial;
  downCoefficient = _downCoefficient;
  upCoefficient = _upCoefficient;
}

RollingAverage::RollingAverage()
{
  value = 5;
  downCoefficient = 0.01;
  upCoefficient = 0.3;
}

float RollingAverage::update(float current)
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  long now =ts.tv_sec * 1000L + ts.tv_nsec/1000000L;


  if( lastUpdate == 0 )
  {  
    lastUpdate = now;
    return value;
  }


  float coefficient;

  if( current > value )
    coefficient = upCoefficient;
  else
    coefficient = downCoefficient;
    
  float delta = (float)(now-lastUpdate) * 0.001; // in secs
  
  value = value * (1.0-coefficient*delta) + current * coefficient * delta;

  lastUpdate = now;
  
  return value;
}

float RollingAverage::get()
{
  return value;
}

void RollingAverage::force(float _value)
{
  value = _value;
}