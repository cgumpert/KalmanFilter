#ifndef BASE_MEASUREMENT_H
#define BASE_MEASUREMENT_H

// STL
#include <iostream>
#include <memory>
//Eigen
#include <Eigen/Dense>

namespace KF
{
  // forward declarations
  template<class S> class BasePredictor;
  template<class S> class StepCache;
  
  /**
     @class CompatibleMeasurement

     This base class provides the generic interface for all specialised measurement classes
     which are able to constrain the state described by the class \c S. It handles the prediction
     of a given state to the next discrete state as well as the update of the predicted state
     using the information from a concrete measurement.
     The prediction is implemented using the visitor pattern to allow the model (specified by
     class \c P = the predictor class) to employ different prediction methods depending on the
     concrete measurement at hand.
     
     @tparam S class describing the state which can be constrained by this measurement
     
     @author Christian Gumpert <christian.gumpert@cern.ch>
   */
  
  template<class S>
  class CompatibleMeasurement
  {
  public:
    /** convenience typedef for shared pointer to State */
    typedef std::shared_ptr<S> sp_S;

    /** convenience method for accessing the dimensionality of the state vector */
    static unsigned int stateDim() {return S::DIM;}
    
    /** perform prediction of state given the predictor and this measurement */
    virtual sp_S acceptPredictor(const BasePredictor<S>&,StepCache<S>&,const S&) const = 0;
    
    /** update the given state due to the information of this measurement */
    virtual sp_S update(const S&) const = 0;

    /** print this measurement in nice formatting */
    virtual void print(std::ostream& os = std::cout) const = 0;

    /** dump this measurement to output */
    virtual void dump(std::ostream&) const = 0;
  };

  /**
     @class BaseMeasurement

     This class acts as base class for all measurements which measure \c mDIM values.
     It implements some of the generic calculations for Kalman Filtering.
     
     @tparam S class describing the state which can be constrained by this measurement
     @tparam mDIM unsigned integer describing the dimensionality of the measurement vector
     
     @author Christian Gumpert <christian.gumpert@cern.ch>
   */
  template<class S, int mDIM>
  class BaseMeasurement : public CompatibleMeasurement<S>
  {
  public:
    /** dimensionality of the state vector */
    static const unsigned int sDIM = S::sDIM;
    /** convenience type definition for the measurement vector */
    typedef Eigen::Matrix<float,mDIM,1> MeasurementVector;
    /** convenience type definition for the measurement covariance */
    typedef Eigen::Matrix<float,mDIM,mDIM> RMatrix;
    /** convenience typedef for shared pointer to State */
    typedef typename CompatibleMeasurement<S>::sp_S sp_S;

    /** perform prediction of state given the predictor and this measurement */
    virtual sp_S acceptPredictor(const BasePredictor<S>&,StepCache<S>&,const S&) const override = 0;

    /** update the given state due to the information of this measurement */    
    virtual sp_S update(const S&) const final;

    /** access the actual measurement */
    virtual MeasurementVector getMeasurementVector() const = 0;

    /** acces the covariance of the measurement */
    virtual RMatrix getCovariance() const = 0;

    /** print this measurement in nice formatting */
    virtual void print(std::ostream& os = std::cout) const override;

    /** dump this measurement to output */
    virtual void dump(std::ostream&) const override;

  protected:
    /** convenience type definition for the state vector */
    typedef Eigen::Matrix<float,sDIM,1>    StateVector;
    /** convenience type definition for the state covariance */
    typedef Eigen::Matrix<float,sDIM,sDIM> StateCovariance;
    /** convenience type definition for the projection matrix */
    typedef Eigen::Matrix<float,mDIM,sDIM> HMatrix;
    /** convenience type definition for the measurement noise transformation matrix */
    typedef Eigen::Matrix<float,mDIM,mDIM> VMatrix;
    /** convenience type definition for the gain matrix */
    typedef Eigen::Matrix<float,sDIM,mDIM> KMatrix;
  
  private:
    /** project a given state onto the measurement frame */
    virtual MeasurementVector projectStateVector(const StateVector&) const = 0;

    /** get the jacobian of the projection with respect to the state vector*/
    virtual HMatrix getH() const = 0;

    /** get the jacobian of the projection with respect to the measurement noise */
    virtual VMatrix getV() const = 0;

    /** get the gain matrix */
    virtual KMatrix getK(const StateCovariance&) const;
  };

  /** overload streaming operator */
  template<class S,unsigned int mDIM>
  std::ostream& operator<<(std::ostream&,const BaseMeasurement<S,mDIM>&);
} // end of namespace

// include implementation
#include "BaseMeasurement.icc"

#endif
