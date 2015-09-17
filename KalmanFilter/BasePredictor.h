#ifndef BASE_PREDICTOR_H
#define BASE_PREDICTOR_H

// STL
#include <memory>
// Eigen
#include <Eigen/Dense>
// KF
#include "StepCache.h"
#include "type_traits_helpers.h"

namespace KF
{
  /**
     @class BasePredictor

     Base class for performing the prediction step of the Kalman filter.

     @tparam S class describing the state which will be propagated
     
     @author Christian Gumpert <christian.gumpert@cern.ch>
   */
  template<class S>
  class BasePredictor
  {
    // make sure that the given template argument is a descendent of KF::BaseState<DIM>
    KF_STATIC_ASSERT_IS_DERIVED_FROM(S,BaseState);

  public:
    /** convenience typedef for shared pointer to State */
    typedef std::shared_ptr<S> sp_S;
    /** convenience type definition for the state vector */    
    typedef typename S::StateVector StateVector;
    /** convenience type definition for the state covariance */
    typedef typename S::StateCovariance StateCovariance;
    /** convenience type definition for the jacobian of the propagation */
    typedef Eigen::Matrix<float,S::sDIM,S::sDIM> TransportJacobian;
    /** convenience type definition for the process noise covariance */
    typedef Eigen::Matrix<float,S::sDIM,S::sDIM> QMatrix;

    /** predict the next state and store result in given cache object */
    template<class T,typename... Args>
    sp_S predict(StepCache<S>&,const S&,const Args&...) const;
  };
} // end of namespace

// include implementation
#include "BasePredictor.icc"

#endif
