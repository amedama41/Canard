#ifndef CANARD_READABLE_PROPERTY_MAP_ADAPTOR_HPP
#define CANARD_READABLE_PROPERTY_MAP_ADAPTOR_HPP

#include <boost/property_map/property_map.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace Canard {
  template <class PropertyMap>
  class readable_property_map_adaptor
    : public boost::put_get_helper<
        typename boost::remove_reference<
          typename boost::property_traits<PropertyMap>::reference
        >::type const&,
        readable_property_map_adaptor<PropertyMap> >
  {
  public:
    typedef typename boost::property_traits<PropertyMap>::key_type key_type;
    typedef typename boost::property_traits<PropertyMap>::value_type value_type;
    typedef typename boost::remove_reference<
      typename boost::property_traits<PropertyMap>::reference
    >::type const& reference;
    typedef boost::readable_property_map_tag category;

    explicit readable_property_map_adaptor(PropertyMap const& pmap)
      : pmap_(pmap)
    { }

    reference operator[](key_type const& key) const
    {
      return get(pmap_, key);
    }

    reference operator()(key_type const& key) const
    {
      return (*this)[key];
    }

  private:
    PropertyMap pmap_;
  };

  template <class PropertyMap>
  readable_property_map_adaptor<PropertyMap>
  convert_readable_property_map(PropertyMap pmap)
  {
    return readable_property_map_adaptor<PropertyMap>(pmap);
  }
}

#endif // CANARD_READABLE_PROPERTY_MAP_ADAPTOR_HPP

