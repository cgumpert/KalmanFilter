#include <list>

#include "KalmanFilter.h"
using namespace KF;
#include "MyMeasurement.h"
#include "State.h"
#include "Predictor.h"

int main()
{
  State s(0.4,-1.2);
  Predictor pred;
  std::list<CompatibleMeasurement<State,Predictor>*> mList;
  mList.push_back(new MyMeasurement(-2));
  mList.push_back(new MyMeasurement(4.5));
  mList.push_back(new MyMeasurement(1.75));
  mList.push_back(new MyMeasurement(7.625));

  KalmanFilter kf;
  kf.filter(s,mList,pred);
  
  return 0;
}
