#ifndef STEP_CACHE_H
#define STEP_CACHE_H

// STL
#include <iostream>
#include <memory>
// Eigen
#include <Eigen/Dense>

namespace KF
{
  template<class S>
  class StepCache
  {
  public:
    typedef Eigen::Matrix<float,S::sDIM,S::sDIM> TransportJacobian;
    typedef std::shared_ptr<const S> sp_cS;
	
    sp_cS getPredictedState() const;
    sp_cS getFilteredState() const;
    sp_cS getSmoothedState() const;
    TransportJacobian getJacobian() const;

    void setPredictedState(const sp_cS& predictedState);
    void setFilteredState(const sp_cS& filteredState);
    void setSmoothedState(const sp_cS& smoothedState);
    void setJacobian(const TransportJacobian& jacobian);

    void dump(std::ostream&) const;
    
  private:
    sp_cS m_predictedState;
    sp_cS m_filteredState;
    sp_cS m_smoothedState;
    TransportJacobian m_jacobian;
  };

  /** overloaded streaming operator */
  template<class S>
  std::ostream& operator<<(std::ostream&,const StepCache<S>&);  
} // end of namespace

// include implementation
#include "StepCache.icc"

#endif
