#ifndef CANARD_PROPERTY_MAP_ITERATOR_HPP
#define CANARD_PROPERTY_MAP_ITERATOR_HPP

#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/mpl/if.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/is_same.hpp>

namespace Canard {
  namespace detail {
    template <class Iterator, class PropertyMap, class Category>
    class property_map_iterator;

    // Proxy Object for WritablePropertyMap
    template <class ReadWritePropertyMap>
    class read_write_property_map_proxy {
      typedef read_write_property_map_proxy<ReadWritePropertyMap> this_type;
      typedef boost::property_traits<ReadWritePropertyMap> PropertyTraits;
      typedef typename PropertyTraits::value_type value_type;
      typedef typename PropertyTraits::key_type   key_type;
      typedef typename PropertyTraits::category   category;

    public:
      read_write_property_map_proxy
      (ReadWritePropertyMap const& m, key_type const& k)
        : m_map(m), m_key(k) { }

      this_type& operator=(value_type const& value)
      {
        put(m_map, m_key, value);
        return *this;
      }

      operator typename PropertyTraits::reference() const
      { return get(m_map, m_key); }

    private:
      ReadWritePropertyMap m_map;
      key_type m_key;
    };

    // Switch for property_map_iterator::dereference_impl
    struct lvalue_type {};
    struct writable_type {};
    struct read_only_type {};

    template <class Iterator, class PropertyMap, class Category>
    struct property_map_iterator_base
    {
      typedef boost::is_same<Category, writable_type> is_writable_type;
      typedef boost::iterator_adaptor<
                property_map_iterator<Iterator, PropertyMap, Category>,
                Iterator,
                typename boost::mpl::if_<
                  is_writable_type,
                  read_write_property_map_proxy<PropertyMap>,
                  typename boost::property_traits<PropertyMap>::value_type
                >::type,
                boost::use_default,
                typename boost::mpl::if_<
                  is_writable_type,
                  read_write_property_map_proxy<PropertyMap>,
                  typename boost::property_traits<PropertyMap>::reference
                >::type
              > type;
    };

    template <class Iterator, class PropertyMap, class Category>
    class property_map_iterator
      : public property_map_iterator_base<Iterator, PropertyMap, Category>::type
    {
      friend class boost::iterator_core_access;

      typedef typename property_map_iterator_base<
                Iterator, PropertyMap, Category>::type super_t;

    public:
      property_map_iterator() { }
      property_map_iterator(Iterator const& it, PropertyMap m)
        : super_t(it),
          m_map(m) { }

    private:
      typename super_t::reference
      dereference_impl(lvalue_type) const
      { return m_map[*(this->base_reference())]; }

      typename super_t::reference
      dereference_impl(writable_type) const
      { return typename super_t::reference(m_map, *(this->base_reference())); }

      typename super_t::reference
      dereference_impl(read_only_type) const
      { return get(m_map, *(this->base_reference())); }

      typename super_t::reference
      dereference() const
      {
        return dereference_impl(Category());
      }

      PropertyMap m_map;
    };
  } // namespace detail

  template <class PropertyMap, class Iterator>
  struct property_map_iterator_generator
  {
    typedef boost::is_convertible<
              typename boost::property_traits<PropertyMap>::category,
              boost::lvalue_property_map_tag
            > is_lvalue_property_map;
    typedef boost::is_convertible<
              typename boost::property_traits<PropertyMap>::category,
              boost::writable_property_map_tag
            > is_writable_property_map;
    typedef typename boost::mpl::if_<
              is_lvalue_property_map,
              detail::lvalue_type,
              typename boost::mpl::if_<
                is_writable_property_map,
                detail::writable_type,
                detail::read_only_type
              >::type
            >::type Category;
    typedef detail::property_map_iterator<Iterator, PropertyMap, Category> type;
  };

  template <class PropertyMap, class Iterator>
  typename property_map_iterator_generator<PropertyMap, Iterator>::type
  make_property_map_iterator(PropertyMap pmap, Iterator iter)
  {
    return typename property_map_iterator_generator<
             PropertyMap, Iterator
           >::type(iter, pmap);
  }

  // Helper function for ReadWritePropertyMap
  // make_property_map_iterator for ReadWritePropertyMap use Proxy Object
  template <class PropertyMap, class Iterator>
  detail::property_map_iterator<PropertyMap, Iterator, detail::read_only_type>
  make_readable_property_map_iterator(PropertyMap pmap, Iterator iter)
  {
    return detail::property_map_iterator<
             PropertyMap, Iterator, detail::read_only_type
           >::type(iter, pmap);
  }
} // namespace Canard

#endif  // CANARD_PROPERTY_MAP_ITERATOR_HPP

