#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// STL
#include <iostream>
#include <fstream>
#include <memory>

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
      std::shared_ptr<S> predicted(0);
      std::shared_ptr<S> filtered(0);
      
      auto m_it = lMeasurements.begin();
      unsigned int k = 0;
      for(; m_it != lMeasurements.end(); ++m_it,++k)
      {
	predicted = (*m_it)->acceptPredictor(predictor,*pCurrent);
	filtered = (*m_it)->update(*predicted);
	pCurrent = filtered.get();

	out << k << "\t" << *predicted << *filtered << std::endl;
      }

      out.close();
    }
  };
}

#endif
