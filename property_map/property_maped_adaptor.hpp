#ifndef CANARD_PROPERTY_MAP_ADAPTOR_HPP
#define CANARD_PROPERTY_MAP_ADAPTOR_HPP

#include <boost/range/adaptor/argument_fwd.hpp>
#include <boost/range/iterator_range.hpp>
#include "property_map_iterator.hpp"

namespace Canard {
  namespace range_detail {
    template <class PropertyMap, class Range>
    struct property_map_range
      : public boost::iterator_range<
                 typename Canard::property_map_iterator_generator<
                   PropertyMap,
                   BOOST_DEDUCED_TYPENAME boost::range_iterator<Range>::type
                 >::type>
    {
    private:
      typedef boost::iterator_range<
                typename property_map_iterator_generator<
                  PropertyMap,
                  BOOST_DEDUCED_TYPENAME boost::range_iterator<Range>::type
                >::type>
              Base;
    public:
        property_map_range(PropertyMap pmap, Range& range)
          : Base(Canard::make_property_map_iterator(pmap, boost::begin(range)),
                 Canard::make_property_map_iterator(pmap, boost::end(range)))
        { }
    };

    template <class T>
    struct property_map_holder : boost::range_detail::holder<T>
    {
      property_map_holder(T r) : boost::range_detail::holder<T>(r) { }
    };

    template <class InputRng, class PropertyMap>
    inline property_map_range<PropertyMap, InputRng>
    operator|(InputRng& r, const property_map_holder<PropertyMap>& p)
    {
      return property_map_range<PropertyMap, InputRng>(p.val, r);
    }

    template <class InputRng, class PropertyMap>
    inline property_map_range<PropertyMap, const InputRng>
    operator|(const InputRng& r, const property_map_holder<PropertyMap>& p)
    {
      return property_map_range<PropertyMap, const InputRng>(p.val, r);
    }
  } // namespace range_detail

  using range_detail::property_map_range;

  namespace adaptors {
    namespace
    {
      const boost::range_detail::forwarder<range_detail::property_map_holder>
        property_maped
          = boost::range_detail::forwarder<range_detail::property_map_holder>();
    }

    template<class InputRange, class PropertyMap>
    inline property_map_range<PropertyMap, InputRange>
    property_map(InputRange& rng, PropertyMap pmap)
    {
      return range_detail::property_map_range<
               PropertyMap, InputRange>(pmap, rng);
    }

    template<class InputRange, class PropertyMap>
    inline property_map_range<PropertyMap, const InputRange>
    property_map(const InputRange& rng, PropertyMap pmap)
    {
      return range_detail::property_map_range<
               PropertyMap, const InputRange>(pmap, rng);
    }
  } // namespace adaptors
} // namespace Canard

#endif  // CANARD_PROPERTY_MAP_ADAPTOR_HPP

