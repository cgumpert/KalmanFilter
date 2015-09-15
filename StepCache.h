#ifndef STEP_CACHE_H
#define STEP_CACHE_H

// STL
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
	
    sp_cS getPredictedState() const {return m_predictedState;}
    sp_cS getFilteredState() const {return m_filteredState;}
    sp_cS getSmoothedState() const {return m_smoothedState;}
    TransportJacobian getJacobian() const {return m_jacobian;}

    void setPredictedState(const sp_cS& predictedState) {m_predictedState = predictedState;}
    void setFilteredState(const sp_cS& filteredState)  {m_filteredState = filteredState;}
    void setSmoothedState(const sp_cS& smoothedState)  {m_smoothedState = smoothedState;}
    void setJacobian(const TransportJacobian& jacobian) {m_jacobian = jacobian;}
    
  private:
    sp_cS m_predictedState;
    sp_cS m_filteredState;
    sp_cS m_smoothedState;
    TransportJacobian m_jacobian;
  };
} // enf of namespace

#endif
