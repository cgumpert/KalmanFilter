#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// STL
#include <iostream>
#include <fstream>
// KF
#include "Measurement.h"

namespace KF
{
  class KalmanFilter
  {
  public:
    template<class S,class P>
    void filter(const S& initialState,const std::list<CompatibleMeasurement<S,P>*>& lMeasurements,const P& predictor) const
    {
      std::ofstream out("filter.log");
      out.precision(5);
      out.setf(std::ios::fixed, std:: ios::floatfield);
    
      const S* pCurrent = &initialState;
      auto m_it = lMeasurements.begin();
      unsigned int k = 0;
      for(; m_it != lMeasurements.end(); ++m_it,++k)
      {
	const S* predicted = (*m_it)->acceptPredictor(predictor,*pCurrent);
	const S* filtered = (*m_it)->update(*predicted);
	pCurrent = filtered;

	out << k << "\t" << *predicted << *filtered << std::endl;
      }

      out.close();
    }
  };
}

#endif
