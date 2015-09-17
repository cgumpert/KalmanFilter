#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// STL
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
    /** test */
    template<class S,class P>
    FilterStack<S> filter(const S& initialState,const std::list<CompatibleMeasurement<S>*>& lMeasurements,const P& predictor) const
    {    
      const S* pCurrent = &initialState;
      std::shared_ptr<S> predicted(0);
      std::shared_ptr<S> filtered(0);
      
      auto m_it = lMeasurements.begin();
      FilterStack<S> stack;
      for(; m_it != lMeasurements.end(); ++m_it)
      {
	StepCache<S> pCache = step(**m_it,*pCurrent,predictor);
	pCurrent = pCache.getFilteredState().get();
	stack.add(std::move(pCache));
      }

      return stack;
    }

    template<class S,class P>
    StepCache<S> step(const CompatibleMeasurement<S>& meas,const S& currentState,const P& predictor) const
    {
      StepCache<S> cache;
      meas.acceptPredictor(predictor,cache,currentState);
      cache.setFilteredState(meas.update(*cache.getPredictedState().get()));

      return cache;
    }

    /** test */
    template<class S>
    void smooth(FilterStack<S>& stack) const
    {
      typedef Eigen::Matrix<float,S::sDIM,S::sDIM> GMatrix;
      typedef std::shared_ptr<S> sp_S;
      typedef typename S::StateVector StateVector;
      typedef typename S::StateCovariance StateCovariance;

      const StepCache<S>* pLast = 0;
      GMatrix G;
      StateVector sv_smoothed;
      StateCovariance cov_smoothed;
      auto it = stack.vector().rbegin();
      for(;it != stack.vector().rend(); ++it)
      {
	if(!pLast)
	{
	  it->setSmoothedState(sp_S(it->getFilteredState()->clone()));
	  pLast = &(*it);
	  continue;
	}

	G = it->getFilteredState()->getCovariance() * it->getJacobian().transpose() * pLast->getPredictedState()->getCovariance().inverse();

	sv_smoothed = it->getFilteredState()->getStateVector() + G * (pLast->getSmoothedState()->getStateVector() - pLast->getPredictedState()->getStateVector());

	cov_smoothed = it->getFilteredState()->getCovariance() - G * (pLast->getPredictedState()->getCovariance() - pLast->getSmoothedState()->getCovariance()) * G.transpose();

	sp_S sp_smoothed(it->getFilteredState()->clone());
	sp_smoothed->setStateVector(sv_smoothed);
	sp_smoothed->setCovariance(cov_smoothed);
	it->setSmoothedState(sp_smoothed);

	pLast = &(*it);
      }
    }
  };
}

#endif
