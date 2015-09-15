#ifndef FILTER_STACK_H
#define FILTER_STACK_H

// KF
#include "StepCache.h"

namespace KF
{
  template<class S>
  class FilterStack
  {
  public:
    void add(StepCache<S>&& sp_step)
    {
      m_vSteps.push_back(std::move(sp_step));
    }
    
  private:
    std::vector<StepCache<S> > m_vSteps;
  };
} // end of namespace

#endif
