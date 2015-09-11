#include "Measurement.h"
using namespace KF;
#include "Predictor.h"
#include "State3D.h"

class XYMeasurement : public Measurement<State3D,Predictor,2>
{
public:
  using Measurement<State3D,Predictor,2>::MeasurementVector;
  using Measurement<State3D,Predictor,2>::MeasurementCovariance;

  XYMeasurement(float x,float y,float dx,float dy,float rho):
    m_measurement(),
    m_R()
  {
    m_measurement << x,y;
    m_R <<
      dx*dx, rho * dx * dy,
      rho * dx * dy, dy*dy;
  };
  
  virtual State3D* acceptPredictor(const Predictor& pred,const State3D& state) const
  {
    return pred.visit(state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

private:
  using Measurement<State3D,Predictor,2>::StateVector;
  using Measurement<State3D,Predictor,2>::StateCovariance;
  using Measurement<State3D,Predictor,2>::HMatrix;
  using Measurement<State3D,Predictor,2>::KMatrix;

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

class YMeasurement : public Measurement<State3D,Predictor,1>
{
public:
  using Measurement<State3D,Predictor,1>::MeasurementVector;
  using Measurement<State3D,Predictor,1>::MeasurementCovariance;

  YMeasurement(float x,float y,float dy):
    m_measurement(),
    m_R(),
    m_x(x)
  {
    m_measurement << y;
    m_R << dy * dy;
  };
  
  virtual State3D* acceptPredictor(const Predictor& pred,const State3D& state) const
  {
    return pred.visit(state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

  float getX() const {return m_x;}

private:
  using Measurement<State3D,Predictor,1>::StateVector;
  using Measurement<State3D,Predictor,1>::StateCovariance;
  using Measurement<State3D,Predictor,1>::HMatrix;
  using Measurement<State3D,Predictor,1>::KMatrix;

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

class XMeasurement : public Measurement<State3D,Predictor,1>
{
public:
  using Measurement<State3D,Predictor,1>::MeasurementVector;
  using Measurement<State3D,Predictor,1>::MeasurementCovariance;

  XMeasurement(float x,float y,float dx):
    m_measurement(),
    m_R(),
    m_y(y)
  {
    m_measurement << x;
    m_R << dx*dx;
  };
  
  virtual State3D* acceptPredictor(const Predictor& pred,const State3D& state) const
  {
    return pred.visit(state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

  float getY() const {return m_y;}

private:
  using Measurement<State3D,Predictor,1>::StateVector;
  using Measurement<State3D,Predictor,1>::StateCovariance;
  using Measurement<State3D,Predictor,1>::HMatrix;
  using Measurement<State3D,Predictor,1>::KMatrix;

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
