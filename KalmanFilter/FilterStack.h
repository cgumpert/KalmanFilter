#ifndef FILTER_STACK_H
#define FILTER_STACK_H

// STL
#include <iostream>
// KF
#include "StepCache.h"
#include "type_traits_helpers.h"

namespace KF
{
  template<class S>
  class FilterStack
  {
    // make sure that the given template argument is a descendent of KF::BaseState<DIM>
    KF_STATIC_ASSERT_IS_DERIVED_FROM(S,BaseState);
    
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
