#ifndef CANARD_TEST_PROPERTY_MAP_HPP
#define CANARD_TEST_PROPERTY_MAP_HPP

#include <boost/property_map/property_map.hpp>

namespace Canard {
  namespace test_detail {
    template <class KeyType>
    struct test_readable_map
    {
      typedef KeyType value_type;
      typedef KeyType reference;
      typedef KeyType key_type;
      typedef boost::readable_property_map_tag category;
    };
    template <class KeyType, class Key>
    typename test_readable_map<KeyType>::reference
    get(test_readable_map<KeyType> map, Key key)
    {
      return key;
    }

    template <class KeyType>
    struct test_writable_map
    {
      typedef KeyType value_type;
      typedef KeyType key_type;
      typedef void    reference;
      typedef boost::writable_property_map_tag category;
      test_writable_map(value_type& v) : value(&v) { }
      value_type* value;
    };
    template <class KeyType, class Key, class Value>
    void put(test_writable_map<KeyType> map, Key, const Value& value)
    {
      *map.value = value;
    }

    template <class KeyType>
    struct test_read_write_map
      : public test_readable_map<KeyType>,
        public test_writable_map<KeyType>
    {
      typedef KeyType value_type;
      typedef KeyType reference;
      typedef KeyType key_type;
      typedef boost::read_write_property_map_tag category;
      test_read_write_map(value_type& v) : test_writable_map<KeyType>(v) { }
    };

    template <class KeyType>
    struct test_lvalue_map
      : public test_read_write_map<KeyType>
    {
      typedef test_read_write_map<KeyType> Base;
      typedef KeyType   value_type;
      typedef KeyType&  reference;
      typedef KeyType   key_type;
      typedef boost::lvalue_property_map_tag category;
      test_lvalue_map(value_type& v) : test_read_write_map<KeyType>(v) { }
      reference operator[](key_type key) const
      { return *this->value; }
    };
  } // namespace test_detail

  using test_detail::test_readable_map;
  using test_detail::test_writable_map;
  using test_detail::test_read_write_map;
  using test_detail::test_lvalue_map;
} // namespace Canard

#endif  // CANARD_TEST_PROPERTY_MAP_HPP

