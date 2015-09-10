#include <list>
#include <random>

#include "XYMeasurement.h"
#include "State3D.h"
#include "KF.h"
#include "Predictor.h"

int main()
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0,0.2);
  
  State3D s(0,0,0.8);
  Predictor pred;
  unsigned int iPoints = 10;
  std::list<CompatibleMeasurement<3,Predictor>*> mList;
  float x,y;
  const float dX = 0.1;
  for(unsigned int i = 1; i <= iPoints; ++i)
  {
    x = i * dX;
    y = x + distribution(generator);
    mList.push_back(new XYMeasurement(x,y));
  }
  
  KalmanFilter kf;
  kf.filter(s,mList,pred);
  
  return 0;
}
