#include <list>
#include <random>

#include "XYMeasurement.h"
#include "State3D.h"
#include "KF.h"
#include "Predictor.h"

int main()
{
  const float dX = 0.1;
  std::default_random_engine generator;
  std::normal_distribution<double> gauss(0,1);
  std::uniform_real_distribution<double> uniform(0,1);
  
  State3D s(0,0,0.8);
  Predictor pred;
  unsigned int iPoints = 5;
  std::list<CompatibleMeasurement<State3D,Predictor>*> mList;
  float x,y,dx,dy,rho;
  float p = 0;
  CompatibleMeasurement<State3D,Predictor>* pMeas = 0;
  // for(unsigned int i = 1; i <= iPoints; ++i)
  // {
  //   x = i * dX;
  //   p = uniform(generator);

  //   if(p < 0.5)
  //   {
  //     std::cout << "XY" << std::endl;
  //     y = x + dX * gauss(generator);
  //     x += 0.1 * dX * gauss(generator);
  //     dx = 0.1 * dX + 0.5 * dX * fabs(gauss(generator));
  //     dy = 0.1 * dX + 0.3 * dX * fabs(gauss(generator));
  //     rho = uniform(generator);
  //     pMeas = new XYMeasurement(x,y,dx,dy,rho);
  //   }
  //   else if(p < 0.75)
  //   {
  //     std::cout << "Y" << std::endl;
  //     y = x + dX * gauss(generator);
  //     dy = 0.1 * dX + 0.3 * dX * fabs(gauss(generator));
  //     pMeas = new YMeasurement(x,y,dy);
  //   }
  //   else
  //   {
  //     std::cout << "X" << std::endl;
  //     y = x;
  //     x += 0.1 * dX * gauss(generator);
  //     dx = 0.1 * dX + 0.5 * dX * fabs(gauss(generator));
  //     pMeas = new XMeasurement(x,y,dx);
  //   }

  //   pMeas->print();
  //   mList.push_back(pMeas);
  // }

  mList.push_back(new XYMeasurement(0.1,0.1,0.02,0.02,0));
  mList.push_back(new XYMeasurement(0.2,0.1,0.02,0.02,0));
  mList.push_back(new XYMeasurement(0.2,0.2,0.02,0.02,0));
  
  KalmanFilter kf;
  kf.filter(s,mList,pred);
  
  return 0;
}
