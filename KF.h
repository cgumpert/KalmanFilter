#ifndef KF_H
#define KF_H

#include <iostream>
#include <list>
#include <Eigen/Dense>

template<int sDIM,int mDIM,class T> class Measurement;

template<int sDIM>
class State
{
public:
  typedef Eigen::Matrix<float,sDIM,1>    StateVector;
  typedef Eigen::Matrix<float,sDIM,sDIM> StateCovariance;

  virtual StateVector getStateVector() const = 0;
  virtual StateCovariance getCovariance() const = 0;
  virtual State<sDIM>* clone() const = 0;
  virtual void print() const;
  virtual void updateStateVector(const StateVector&) = 0;
  virtual void updateCovariance(const StateCovariance&) = 0;
};

template<int sDIM>
void State<sDIM>::print() const
{
  std::cout << "state vector = " << std::endl << getStateVector() << std::endl;
  std::cout << "covariance = " << std::endl << getCovariance() << std::endl;
}

template<int sDIM,class T>
class CompatibleMeasurement
{
public:
  typedef Eigen::Matrix<float,sDIM,1>    StateVector;

  virtual State<sDIM>* update(const State<sDIM>&) const = 0;
  virtual State<sDIM>* acceptPredictor(const T&,const State<sDIM>&) const = 0;
  virtual void print() const = 0;
};

template<int sDIM,int mDIM,class T>
class Measurement : public CompatibleMeasurement<sDIM,T>
{
public:
  typedef Eigen::Matrix<float,mDIM,1>    MeasurementVector;

  virtual State<sDIM>* update(const State<sDIM>&) const;
  virtual State<sDIM>* acceptPredictor(const T&,const State<sDIM>&) const = 0;
  virtual MeasurementVector getMeasurementVector() const = 0;
  virtual void print() const
  {
    std::cout << "m = " << std::endl << getMeasurementVector() << std::endl;
  }
  
protected:
  typedef Eigen::Matrix<float,sDIM,1>    StateVector;
  typedef Eigen::Matrix<float,sDIM,sDIM> StateCovariance;
  typedef Eigen::Matrix<float,mDIM,sDIM> HMatrix;
  typedef Eigen::Matrix<float,mDIM,mDIM> RMatrix;
  typedef Eigen::Matrix<float,sDIM,mDIM> KMatrix;
  
private:
  virtual MeasurementVector projectStateVector(const StateVector&) const = 0;
  virtual HMatrix getH() const = 0;
  virtual RMatrix getR() const = 0;
  virtual KMatrix getK(const StateCovariance&) const;
};

template<int sDIM,int mDIM,class T>
State<sDIM>* Measurement<sDIM,mDIM,T>::update(const State<sDIM>& state) const
{
  StateVector x_pred = state.getStateVector();
  StateCovariance cov_pred = state.getCovariance();

  KMatrix K = getK(cov_pred);

  // update state vector
  MeasurementVector z = getMeasurementVector();
  MeasurementVector x_proj = projectStateVector(x_pred);
  StateVector x_filtered = x_pred + K * (z - x_proj);

  // update covariance
  HMatrix H = getH();
  StateCovariance unit; unit.setIdentity();
  StateCovariance cov_filtered = (unit - K * H) * cov_pred;

  State<sDIM>* filtered = state.clone();
  filtered->updateStateVector(x_filtered);
  filtered->updateCovariance(cov_filtered);

  return filtered;
}

template<int sDIM,int mDIM,class T>
typename Measurement<sDIM,mDIM,T>::KMatrix Measurement<sDIM,mDIM,T>::getK(const StateCovariance& P) const
{
  HMatrix H = getH();
  RMatrix R = getR();
  
  return P * H.transpose() * (H * P * H.transpose() + R).inverse();
};

class KalmanFilter
{
public:
  template<int sDIM,class T>
  void filter(const State<sDIM>& initialState,const std::list<CompatibleMeasurement<sDIM,T>*>& lMeasurements,const T& predictor) const
  {
    const State<sDIM>* pCurrent = &initialState;
    auto m_it = lMeasurements.begin();
    unsigned int k = 0;
    for(; m_it != lMeasurements.end(); ++m_it,++k)
    {
      std::cout << k << ".iteration: " << std::endl;
      pCurrent->print();
      (*m_it)->print();
      const State<sDIM>* predicted = (*m_it)->acceptPredictor(predictor,*pCurrent);
      std::cout << "predicted:" << std::endl;
      predicted->print();
      const State<sDIM>* filtered = (*m_it)->update(*predicted);
      std::cout << "filtered:" << std::endl;
      filtered->print();
      pCurrent = filtered;
      std::cout << std::endl;
    }
  }
};

#endif
