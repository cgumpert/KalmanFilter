#include "KF.h"
#include "Predictor.h"

class XYMeasurement : public Measurement<3,2,Predictor>
{
public:
  using Measurement<3,2,Predictor>::MeasurementVector;

  XYMeasurement(float x,float y):
    m_measurement()
  {
    m_measurement << x,y;
  };
  
  virtual State<3>* acceptPredictor(const Predictor& pred,const State<3>& state) const
  {
    return pred.visit(state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

private:
  using Measurement<3,2,Predictor>::StateVector;
  using Measurement<3,2,Predictor>::StateCovariance;
  using Measurement<3,2,Predictor>::HMatrix;
  using Measurement<3,2,Predictor>::RMatrix;
  using Measurement<3,2,Predictor>::KMatrix;

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
    static RMatrix R;
    R <<
      0.1,0,
      0,0.3;
    return R;
  }

  MeasurementVector m_measurement;
};
