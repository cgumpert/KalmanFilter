#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// STL
#include <iostream>
#include <fstream>
#include <memory>

// KF
#include "BaseMeasurement.h"
#include "StepCache.h"
#include "FilterStack.h"

namespace KF
{
  class KalmanFilter
  {
  public:
    template<class S,class P>
    void filter(const S& initialState,const std::list<CompatibleMeasurement<S>*>& lMeasurements,const P& predictor) const
    {    
      const S* pCurrent = &initialState;
      std::shared_ptr<S> predicted(0);
      std::shared_ptr<S> filtered(0);
      
      auto m_it = lMeasurements.begin();
      FilterStack<S> stack;
      unsigned int k = 0;
      for(; m_it != lMeasurements.end(); ++m_it, ++k)
      {
	std::cout << k << std::endl;
	StepCache<S> pCache = step(**m_it,*pCurrent,predictor);
	pCurrent = pCache.getFilteredState().get();
	stack.add(std::move(pCache));
      }
    }

    template<class S,class P>
    StepCache<S> step(const CompatibleMeasurement<S>& meas,const S& currentState,const P& predictor) const
      {
	StepCache<S> cache;
	meas.acceptPredictor(predictor,cache,currentState);
	cache.setFilteredState(meas.update(*cache.getPredictedState().get()));

	return cache;
      }
  };
}

#endif
