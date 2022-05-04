
#ifndef ROLLING_AVERAGE_H
#define ROLLING_AVERAGE_H

class RollingAverage
{
  public:
  RollingAverage();
  RollingAverage(float initial, float _downCoefficient, float _upCoefficient);
  float upCoefficient;
  float downCoefficient;
  float value;
  long lastUpdate = 0;

  float update(float current);
  float get();
  void force(float value);
  
};

#endif
