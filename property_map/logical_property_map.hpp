#ifndef CANARD_LOGICAL_PROPERTY_MAP_H
#define CANARD_LOGICAL_PROPERTY_MAP_H

#include <boost/property_map/property_map.hpp>

namespace Canard {
  template <class PredictMap>
  class not_predict_map
    : public boost::put_get_helper<bool, not_predict_map<PredictMap> >
  {
  public:
    typedef typename PredictMap::key_type key_type;
    typedef bool value_type;
    typedef bool reference;
    typedef boost::readable_property_map_tag category;

    explicit not_predict_map(PredictMap const& pmap)
      : pmap(pmap)
    {
    }

    reference operator[](key_type const& key) const
    {
      return !get(pmap, key);
    }

  private:
    PredictMap pmap;
  };

  template <class PredictMap>
  inline not_predict_map<PredictMap>
  make_not_predict_map(PredictMap const& pmap)
  {
    return not_predict_map<PredictMap>(pmap);
  }


  template <class PredictMap1, class PredictMap2>
  class and_predict_map
    : public boost::put_get_helper<
        bool,
        and_predict_map<PredictMap1, PredictMap2>
      >
  {
  public:
    typedef boost::readable_property_map_tag category;
    typedef bool value_type;
    typedef bool reference;
    typedef typename PredictMap1::key_type key_type;

    and_predict_map(PredictMap1 const& pmap1, PredictMap2 const& pmap2)
      : pmap1(pmap1), pmap2(pmap2)
    {
    }

    reference operator[](key_type const& key) const
    {
      return get(pmap1, key) && get(pmap2, key);
    }

  private:
    PredictMap1 pmap1;
    PredictMap2 pmap2;
  };

  template <class PredictMap1, class PredictMap2>
  inline and_predict_map<PredictMap1, PredictMap2>
  make_and_predict_map(PredictMap1 const& pmap1, PredictMap2 const& pmap2)
  {
    return and_predict_map<PredictMap1, PredictMap2>(pmap1, pmap2);
  }

  template <class PredictMap1, class PredictMap2>
  class or_predict_map
    : public boost::put_get_helper<
        bool,
        or_predict_map<PredictMap1, PredictMap2>
      >
  {
  public:
    typedef boost::readable_property_map_tag category;
    typedef bool value_type;
    typedef bool reference;
    typedef typename PredictMap1::key_type key_type;

    or_predict_map(PredictMap1 const& pmap1, PredictMap2 const& pmap2)
      : pmap1(pmap1), pmap2(pmap2)
    {
    }

    reference operator[](key_type const& key) const
    {
      return get(pmap1, key) || get(pmap2, key);
    }

  private:
    PredictMap1 pmap1;
    PredictMap2 pmap2;
  };

  template <class PredictMap1, class PredictMap2>
  inline or_predict_map<PredictMap1, PredictMap2>
  make_or_predict_map(PredictMap1 const& pmap1, PredictMap2 const& pmap2)
  {
    return or_predict_map<PredictMap1, PredictMap2>(pmap1, pmap2);
  }
} // namespace Canard

#endif  // CANARD_LOGICAL_PROPERTY_MAP_H

