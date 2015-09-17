#include <iostream>
#include <list>
#include <random>

#include "KalmanFilter.h"
using namespace KF;
#include "XYMeasurement.h"
#include "State3D.h"
#include "Predictor.h"

int main()
{
  const float dX = 0.1;
  const unsigned int iPoints = 50000;

  std::default_random_engine generator;
  std::normal_distribution<double> gauss(0,1);
  std::uniform_real_distribution<double> uniform(0,1);
  
  State3D s(0,0,0.8);
  Predictor pred;
  std::list<CompatibleMeasurement<State3D>*> mList;
  float x,y,dx,dy;
  float p = 0;
  CompatibleMeasurement<State3D>* pMeas = 0;
  for(unsigned int i = 1; i <= iPoints; ++i)
  {
    x = i * dX;
    p = uniform(generator);

    if(p < 0.5)
    {
      y = x + dX * gauss(generator);
      x += 0.1 * dX * gauss(generator);
      dx = 0.1 * dX + 0.5 * dX * fabs(gauss(generator));
      pMeas = new XMeasurement(x,y,dx);
    }
    else
    {
      y = x + dX * gauss(generator);
      dy = 0.1 * dX + 0.3 * dX * fabs(gauss(generator));
      pMeas = new YMeasurement(x,y,dy);
    }

    mList.push_back(pMeas);
  }

  KalmanFilter kf;
  FilterStack<State3D> stack = kf.filter(s,mList,pred);
  kf.smooth(stack);
  
  return 0;
}
