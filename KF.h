#ifndef KF_H
#define KF_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>
#include <Eigen/Dense>

template<unsigned int DIM>
class State
{
public:
  static const unsigned int sDIM = DIM;
  typedef Eigen::Matrix<float,sDIM,1>    StateVector;
  typedef Eigen::Matrix<float,sDIM,sDIM> StateCovariance;

  virtual StateVector getStateVector() const = 0;
  virtual StateCovariance getCovariance() const = 0;
  virtual State<sDIM>* clone() const = 0;
  virtual void print() const;
  virtual void updateStateVector(const StateVector&) = 0;
  virtual void updateCovariance(const StateCovariance&) = 0;
};

template<unsigned int sDIM>
void State<sDIM>::print() const
{
  std::cout << "state vector = " << std::endl << getStateVector() << std::endl;
  std::cout << "covariance = " << std::endl << getCovariance() << std::endl;
}

template<unsigned int sDIM>
std::ostream& operator<<(std::ostream& os,const State<sDIM>& state)
{
  typename State<sDIM>::StateVector sv = state.getStateVector();
  typename State<sDIM>::StateCovariance cov = state.getCovariance();
  
  for(unsigned int i = 0; i < sDIM; ++i)
    os << std::setw(8) << sv[i] << "\t" << sqrt(cov(i,i)) << "\t";
  
  return os;
}

template<class S,class T>
class CompatibleMeasurement
{
public:
  static const unsigned int sDIM = S::sDIM;
  typedef Eigen::Matrix<float,sDIM,1>    StateVector;

  virtual S* update(const S&) const = 0;
  virtual S* acceptPredictor(const T&,const S&) const = 0;
  virtual void print() const = 0;
};

template<class S,int mDIM,class T>
class Measurement : public CompatibleMeasurement<S,T>
{
public:
  static const unsigned int sDIM =  S::sDIM;
  typedef Eigen::Matrix<float,mDIM,1>    MeasurementVector;

  virtual S* update(const S&) const;
  virtual S* acceptPredictor(const T&,const S&) const = 0;
  virtual MeasurementVector getMeasurementVector() const = 0;
  virtual void print() const
  {
    std::cout << "m = " << std::endl << getMeasurementVector() << std::endl;
    std::cout << "R = " << std::endl << getR() << std::endl;
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

template<class S,int mDIM,class T>
S* Measurement<S,mDIM,T>::update(const S& state) const
{
  StateVector x_pred = state.getStateVector();
  StateCovariance cov_pred = state.getCovariance();

  KMatrix K = getK(cov_pred);

  std::cout << "cov_pred = " << std::endl << cov_pred << std::endl;
  std::cout << "K = " << std::endl << K << std::endl;
  std::cout << "K * H = " << std::endl << K * getH() << std::endl;
  // update state vector
  MeasurementVector z = getMeasurementVector();
  MeasurementVector x_proj = projectStateVector(x_pred);
  StateVector x_filtered = x_pred + K * (z - x_proj);

  // update covariance
  HMatrix H = getH();
  StateCovariance unit; unit.setIdentity();
  StateCovariance cov_filtered = (unit - K * H) * cov_pred;
  std::cout << "cov_filtered = " << std::endl << cov_filtered << std::endl;

  S* filtered = state.clone();
  filtered->updateStateVector(x_filtered);
  filtered->updateCovariance(cov_filtered);

  return filtered;
}

template<class S,int mDIM,class T>
typename Measurement<S,mDIM,T>::KMatrix Measurement<S,mDIM,T>::getK(const StateCovariance& P) const
{
  HMatrix H = getH();
  RMatrix R = getR();
  
  return P * H.transpose() * (H * P * H.transpose() + R).inverse();
};

class KalmanFilter
{
public:
  template<class S,class T>
  void filter(const S& initialState,const std::list<CompatibleMeasurement<S,T>*>& lMeasurements,const T& predictor) const
  {
    std::ofstream out("filter.log");
    out.precision(5);
    out.setf(std::ios::fixed, std:: ios::floatfield);
    
    const S* pCurrent = &initialState;
    auto m_it = lMeasurements.begin();
    unsigned int k = 0;
    for(; m_it != lMeasurements.end(); ++m_it,++k)
    {
      // std::cout << k << ".iteration: " << std::endl;
      // pCurrent->print();
      // (*m_it)->print();
      const S* predicted = (*m_it)->acceptPredictor(predictor,*pCurrent);
      // std::cout << "predicted:" << std::endl;
      // predicted->print();
      const S* filtered = (*m_it)->update(*predicted);
      // std::cout << "filtered:" << std::endl;
      // filtered->print();
      pCurrent = filtered;
      // std::cout << std::endl;

      out << k << "\t" << *predicted << *filtered << std::endl;
    }

    out.close();
  }
};

#endif
