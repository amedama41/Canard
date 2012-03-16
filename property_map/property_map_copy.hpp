#ifndef CANARD_PROPERTY_MAP_COPY_HPP
#define CANARD_PROPERTY_MAP_COPY_HPP

#include <boost/property_map/property_map.hpp>

namespace Canard {
  template <class PutPropMap, class GetPropMap>
  class property_map_copy
  {
  public:
    typedef void result_type;
    typedef typename boost::property_traits<PutPropMap>::key_type param_type;

    property_map_copy(PutPropMap const& ppm, GetPropMap const& gpm)
      : put_pm(ppm), get_pm(gpm)
    {
      typedef typename boost::property_traits<GetPropMap>::key_type get_key;
      typedef typename boost::property_traits<GetPropMap>::key_type put_key;
      BOOST_CONCEPT_ASSERT((boost::Convertible<get_key, put_key>));
      typedef typename boost::property_traits<GetPropMap>::value_type get_value;
      typedef typename boost::property_traits<GetPropMap>::value_type put_value;
      BOOST_CONCEPT_ASSERT((boost::Convertible<get_value, put_value>));
    }

    result_type operator()(param_type const& key) const
    {
      put(put_pm, k, get(get_pm, key));
    }

  private:
    PutPropMap put_pm;
    GetPropMap get_pm;
  };

  template <class PutPropMap, class GetPropMap>
  inline property_map_copy<PutPropMap, GetPropMap>
  make_property_map_copy(PutPropMap const& ppm, GetPropMap const& gpm)
  {
    return property_map_copy<PutPropMap, GetPropMap>(ppm, gpm);
  }
}

#endif // CANARD_PROPERTY_MAP_COPY_HPP

