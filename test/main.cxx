#include <list>
#include <iostream>

#include "KalmanFilter.h"
#include "FilterStack.h"
using namespace KF;
#include "MyMeasurement.h"
#include "State.h"
#include "Predictor.h"

int main()
{
  State s(0.4,-1.2);
  Predictor pred;
  std::list<CompatibleMeasurement<State>*> mList;
  mList.push_back(new MyMeasurement(-2));
  mList.push_back(new MyMeasurement(4.5));
  mList.push_back(new MyMeasurement(1.75));
  mList.push_back(new MyMeasurement(7.625));

  KalmanFilter kf;
  FilterStack<State> stack = kf.filter(s,mList,pred);
  kf.smooth(stack);
  stack.dump(std::cout);
  
  return 0;
}
