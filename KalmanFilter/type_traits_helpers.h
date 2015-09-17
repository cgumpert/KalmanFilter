#ifndef TYPE_TRAITS_HELPERS_H
#define TYPE_TRAITS_HELPERS_H

#define KF_IS_DERIVED_FROM_STRUCT(BASE)                                                      \
                                                                                             \
  template<class T>                                                                          \
  struct is_derived_from_##BASE                                                              \
  {                                                                                          \
    enum {value = std::is_convertible<T,is_derived_from_##BASE##_impl<BASE> >::value};       \
  };                                                                                         \

#define KF_STATIC_ASSERT_IS_DERIVED_FROM(DERIVED,BASE)                                       \
  static_assert(is_derived_from_##BASE<DERIVED>::value,"template parameter <class " #DERIVED \
    "> is not derived from <class " #BASE ">");


namespace KF
{

  // forward declarations
  template<unsigned int> class BaseState;
  template<class,unsigned int> class BaseMeasurement;
  template<class> class BasePredictor;

  template<template<unsigned int> class Base>
  struct is_derived_from_BaseState_impl
  {
    template<unsigned int D>
    is_derived_from_BaseState_impl(const Base<D>&);
  };
  
  template<template<class,unsigned int> class Base>
  struct is_derived_from_BaseMeasurement_impl
  {
    template<class S,unsigned int D>
    is_derived_from_BaseMeasurement_impl(const Base<S,D>&);
  };
  
  template<template<class> class Base>
  struct is_derived_from_BasePredictor_impl
  {
    template<class S>
    is_derived_from_BasePredictor_impl(const Base<S>&);
  };

  KF_IS_DERIVED_FROM_STRUCT(BaseState);
  KF_IS_DERIVED_FROM_STRUCT(BaseMeasurement);
  KF_IS_DERIVED_FROM_STRUCT(BasePredictor);
  
} // end of namespace

#endif
