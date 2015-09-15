#ifndef FILTER_STACK_H
#define FILTER_STACK_H

// STL
#include <iostream>
// KF
#include "StepCache.h"

namespace KF
{
  template<class S>
  class FilterStack
  {
  public:
    void add(StepCache<S>&&);
    
    void dump(std::ostream& os) const;

    std::vector<StepCache<S> >& vector() {return m_vSteps;}
  private:
    std::vector<StepCache<S> > m_vSteps;
  };

  /** overloaded streaming operator */
  template<class S>
  std::ostream& operator<<(std::ostream&,const FilterStack<S>&);
} // end of namespace

// include implementation
#include "FilterStack.icc"

#endif
