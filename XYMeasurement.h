#include "KF.h"
#include "Predictor.h"
#include "State3D.h"

class XYMeasurement : public Measurement<State3D,2,Predictor>
{
public:
  using Measurement<State3D,2,Predictor>::MeasurementVector;

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
  using Measurement<State3D,2,Predictor>::StateVector;
  using Measurement<State3D,2,Predictor>::StateCovariance;
  using Measurement<State3D,2,Predictor>::HMatrix;
  using Measurement<State3D,2,Predictor>::RMatrix;
  using Measurement<State3D,2,Predictor>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H <<
      1,0,0,
      0,1,0;
    return H;
  }
  
  virtual RMatrix getR() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  RMatrix m_R;
};

class YMeasurement : public Measurement<State3D,1,Predictor>
{
public:
  using Measurement<State3D,1,Predictor>::MeasurementVector;

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
  using Measurement<State3D,1,Predictor>::StateVector;
  using Measurement<State3D,1,Predictor>::StateCovariance;
  using Measurement<State3D,1,Predictor>::HMatrix;
  using Measurement<State3D,1,Predictor>::RMatrix;
  using Measurement<State3D,1,Predictor>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H << 0,1,0;

    return H;
  }
  
  virtual RMatrix getR() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  RMatrix m_R;
  float m_x;
};

class XMeasurement : public Measurement<State3D,1,Predictor>
{
public:
  using Measurement<State3D,1,Predictor>::MeasurementVector;

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
  using Measurement<State3D,1,Predictor>::StateVector;
  using Measurement<State3D,1,Predictor>::StateCovariance;
  using Measurement<State3D,1,Predictor>::HMatrix;
  using Measurement<State3D,1,Predictor>::RMatrix;
  using Measurement<State3D,1,Predictor>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H << 1,0,0;

    return H;
  }
  
  virtual RMatrix getR() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  RMatrix m_R;
  float m_y;
};
