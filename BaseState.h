#ifndef BASE_STATE_H
#define BASE_STATE_H

// STL
#include <iostream>
// Eigen
#include <Eigen/Dense>

namespace KF
{
  /**
     @class BaseState

     base class describing the interface for a generic state vector

     @tparam DIM dimensionality of the state vector (>0)
     
     @author Christian Gumpert <christian.gumpert@cern.ch>
  */
  template<unsigned int DIM>
  class BaseState
  {
  public:
    /** let derived classes access the dimensionality */
    static const unsigned int sDIM = DIM;
    /** convenience type definition for the state vector */
    typedef Eigen::Matrix<float,sDIM,1>    StateVector;
    /** convenience type definition for the state covariance */
    typedef Eigen::Matrix<float,sDIM,sDIM> StateCovariance;
    /** convenience type definition for the jacobian */
    typedef Eigen::Matrix<float,sDIM,sDIM> TransportJacobian;
    /** convenience type definition for the process noise */
    typedef Eigen::Matrix<float,sDIM,sDIM> QMatrix;

    /** access the state vector */
    virtual StateVector getStateVector() const = 0;

    /** access the covariance of the state vector */
    virtual StateCovariance getCovariance() const = 0;

    /** set the state vector */
    virtual void setStateVector(const StateVector&) = 0;

    /** set the covariance of the state vector */
    virtual void setCovariance(const StateCovariance&) = 0;
    
    /** virtual factory method */
    virtual BaseState<sDIM>* clone() const = 0;

    /** print information about current state in nice format*/
    virtual void print(std::ostream& os = std::cout) const;

    /** dump information about current state */
    virtual void dump(std::ostream&) const;

    /** update covariance matrix given jacobian of the prediction and process noise */
    void updateCovariance(const TransportJacobian& jacobian,const QMatrix* pNoise = 0);
  };

  /** overloaded streaming operator */
  template<unsigned int DIM>
  std::ostream& operator<<(std::ostream&,const BaseState<DIM>&);
}

// include implementations
#include "BaseState.icc"

#endif
