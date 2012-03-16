#ifndef CANARD_VECOTOR_PROPERTY_MAP_HPP
#define CANARD_VECOTOR_PROPERTY_MAP_HPP

#include <boost/property_map/property_map.hpp>
#include <vector>
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
                vector_property_map<T, IndexMap, Alloc> >
  {
    typedef std::vector<T, Alloc> Storage;
    typedef typename boost::property_traits<IndexMap>::value_type index_type;

  public:
    typedef T value_type;
    typedef typename boost::property_traits<IndexMap>::key_type key_type;
    typedef typename std::iterator_traits<
        typename std::vector<T, Alloc>::iterator
      >::reference reference;
    typedef typename boost::mpl::if_<
      boost::is_const<T>,
      boost::readable_property_map_tag,
      boost::lvalue_property_map_tag>::type category;

  public:
    explicit vector_property_map(IndexMap const& i_map = IndexMap())
      : store_(boost::make_shared<Storage>()), index_map_(i_map)
    { }

    vector_property_map
    (std::size_t initial_size, IndexMap const& i_map = IndexMap())
      : store_(boost::make_shared<Storage>(initial_size)), index_map_(i_map)
    { }

    void resize(std::size_t size)
    { store_->resize(size); }

    reference operator[](const key_type& key) const
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

