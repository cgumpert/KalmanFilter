// STL
#include <memory>

// KF
#include "BaseMeasurement.h"
#include "BasePredictor.h"
#include "State3D.h"
#include "Predictor.h"
#include "FilterStack.h"
using namespace KF;

class XYMeasurement : public BaseMeasurement<State3D,Predictor,2>
{
public:
  using BaseMeasurement<State3D,Predictor,2>::MeasurementVector;
  using BaseMeasurement<State3D,Predictor,2>::MeasurementCovariance;
  using BaseMeasurement<State3D,Predictor,2>::Predictor;
  using BaseMeasurement<State3D,Predictor,2>::Cache;

  XYMeasurement(float x,float y,float dx,float dy,float rho):
    m_measurement(),
    m_R()
  {
    m_measurement << x,y;
    m_R <<
      dx*dx, rho * dx * dy,
      rho * dx * dy, dy*dy;
  };
  
  virtual void acceptPredictor(const Predictor& pred,const State3D& state,Cache& cache) const override
  {
    return pred.visit(cache,*this,0.3,true);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

private:
  using BaseMeasurement<State3D,Predictor,2>::StateVector;
  using BaseMeasurement<State3D,Predictor,2>::StateCovariance;
  using BaseMeasurement<State3D,Predictor,2>::HMatrix;
  using BaseMeasurement<State3D,Predictor,2>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H <<
      1,0,0,
      0,1,0;
    return H;
  }
  
  virtual MeasurementCovariance getCovariance() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  MeasurementCovariance m_R;
};

class YMeasurement : public BaseMeasurement<State3D,Predictor,1>
{
public:
  using BaseMeasurement<State3D,Predictor,1>::MeasurementVector;
  using BaseMeasurement<State3D,Predictor,1>::MeasurementCovariance;

  YMeasurement(float x,float y,float dy):
    m_measurement(),
    m_R(),
    m_x(x)
  {
    m_measurement << y;
    m_R << dy * dy;
  };

  virtual void acceptPredictor(const Predictor& pred,const State3D& state,Cache& cache) const override
  {
    return pred.visit(cache,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

  float getX() const {return m_x;}

private:
  using BaseMeasurement<State3D,Predictor,1>::StateVector;
  using BaseMeasurement<State3D,Predictor,1>::StateCovariance;
  using BaseMeasurement<State3D,Predictor,1>::HMatrix;
  using BaseMeasurement<State3D,Predictor,1>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H << 0,1,0;

    return H;
  }
  
  virtual MeasurementCovariance getCovariance() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  MeasurementCovariance m_R;
  float m_x;
};

class XMeasurement : public BaseMeasurement<State3D,Predictor,1>
{
public:
  using BaseMeasurement<State3D,Predictor,1>::MeasurementVector;
  using BaseMeasurement<State3D,Predictor,1>::MeasurementCovariance;

  XMeasurement(float x,float y,float dx):
    m_measurement(),
    m_R(),
    m_y(y)
  {
    m_measurement << x;
    m_R << dx*dx;
  };
  
  virtual void acceptPredictor(const Predictor& pred,const State3D& state,Cache& cache) const override
  {
    return pred.visit(cache,state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

  float getY() const {return m_y;}

private:
  using BaseMeasurement<State3D,Predictor,1>::StateVector;
  using BaseMeasurement<State3D,Predictor,1>::StateCovariance;
  using BaseMeasurement<State3D,Predictor,1>::HMatrix;
  using BaseMeasurement<State3D,Predictor,1>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H << 1,0,0;

    return H;
  }
  
  virtual MeasurementCovariance getCovariance() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  MeasurementCovariance m_R;
  float m_y;
};
