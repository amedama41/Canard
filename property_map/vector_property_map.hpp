#ifndef CANARD_VECOTOR_PROPERTY_MAP_HPP
#define CANARD_VECOTOR_PROPERTY_MAP_HPP

#include <vector>
#include <boost/property_map/property_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/is_const.hpp>

namespace Canard {
  template <class T, class IndexMap, class Alloc = std::allocator<T> >
  class vector_property_map
    : public boost::put_get_helper<
                typename std::iterator_traits<
                    typename std::vector<T, Alloc>::iterator
                >::reference,
                vector_property_map<T, IndexMap, Alloc>
             >
  {
    typedef std::vector<T, Alloc> Storage;
    typedef typename boost::property_traits<IndexMap>::value_type index_type;

  public:
    typedef T value_type;
    typedef typename std::iterator_traits<
        typename std::vector<T, Alloc>::iterator
      >::reference reference;
    typedef typename boost::property_traits<IndexMap>::key_type key_type;
    typedef typename boost::mpl::if_<
      boost::is_const<T>,
      boost::readable_property_map_tag,
      boost::lvalue_property_map_tag
    >::type category;

  public:
    vector_property_map()
      : store_(boost::make_shared<Storage>()),
        index_map_(IndexMap())
    {
    }

    explicit vector_property_map(IndexMap const& i_map)
      : store_(boost::make_shared<Storage>()),
        index_map_(i_map)
    {
    }

    explicit vector_property_map(std::size_t initial_size)
      : store_(boost::make_shared<Storage>(initial_size)),
        index_map_(IndexMap())
    {
    }

    vector_property_map(std::size_t initial_size, IndexMap const& i_map)
      : store_(boost::make_shared<Storage>(initial_size)),
        index_map_(i_map)
    {
    }

    void resize(std::size_t new_size)
    {
      store_->resize(new_size);
    }

    void resize(std::size_t new_size, value_type const& value)
    {
      store_->resize(new_size, value);
    }

    reference operator[](key_type const& key) const
    {
      index_type const i = get(index_map_, key);
      if (static_cast<std::size_t>(i) >= store_->size()) {
        store_->resize(i + 1);
      }
      return (*store_)[i];
    }

  private:
    boost::shared_ptr<Storage> store_;
    IndexMap index_map_;
  };

  template <class T, class Alloc = std::allocator<T>, class IndexMap>
  vector_property_map<T, IndexMap, Alloc>
  make_vector_property_map(IndexMap index_map)
  {
    return vector_property_map<T, IndexMap, Alloc>(index_map);
  }

  template <class T, class Alloc = std::allocator<T>, class IndexMap>
  vector_property_map<T, IndexMap, Alloc>
  make_vector_property_map(std::size_t initial_size, IndexMap index_map)
  {
    return vector_property_map<T, IndexMap, Alloc>(initial_size, index_map);
  }
} // namespace Canard

#endif  // CANARD_VECOTOR_PROPERTY_MAP_HPP

