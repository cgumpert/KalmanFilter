#ifndef BASE_PREDICTOR_H
#define BASE_PREDICTOR_H

// STL
#include <memory>
#include <iostream>
// Eigen
#include <Eigen/Dense>
// KF
#include "StepCache.h"

namespace KF
{  
  template<class S>
  class BasePredictor
  {
  public:
    typedef std::shared_ptr<S> sp_S;
    typedef Eigen::Matrix<float,S::sDIM,S::sDIM> TransportJacobian;
    typedef Eigen::Matrix<float,S::sDIM,S::sDIM> QMatrix;
    typedef typename S::StateVector StateVector;
    typedef typename S::StateCovariance StateCovariance;
    
    template<class T,typename... Args>
    void predict(StepCache<S>& cache,const S& inState,const Args&... args) const
    {
      const StateVector& newSV   = static_cast<const T*>(this)->propagate(inState,args...);
      const TransportJacobian& J = static_cast<const T*>(this)->getTransportJacobian(inState,args...);
      const QMatrix& Q           = static_cast<const T*>(this)->getProcessNoise(inState,args...);
      StateCovariance pred_cov   = J * inState.getCovariance() * J.transpose() + Q;
      
      std::shared_ptr<S> sp_predicted(inState.clone());
      sp_predicted->setStateVector(newSV);
      sp_predicted->setCovariance(pred_cov);
      cache.setPredictedState(sp_predicted);
    }
  };
} // end of namespace

#endif
