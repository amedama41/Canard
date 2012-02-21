#ifndef CANARD_DUMMY_CENTER_MAP_H
#define CANARD_DUMMY_CENTER_MAP_H

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

namespace CanardGraph {
  template <class Graph, class PropMap>
    struct DummyCenterMap
    : boost::put_get_helper<bool, DummyCenterMap<Graph, PropMap> >
    {
      typedef bool value_type;
      typedef bool reference;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor key_type;
      typedef boost::readable_property_map_tag category;

      DummyCenterMap(const Graph& g, const PropMap& pm) : g(g), pm(pm) {}
      value_type operator[](key_type v) const
      {
        using boost::out_degree;
        if (get(pm, v)) {
          return true;
        }
        return out_degree(v, g) != 2;
      }
      value_type operator()(key_type v) const
      {
        return (*this)[v];
      }
      const Graph& g;
      PropMap pm;
    };

  template <class Graph, class PropMap>
    const DummyCenterMap<Graph, PropMap>
    make_dummy_center_map(const Graph& g, const PropMap pm)
    {
      return DummyCenterMap<Graph, PropMap>(g, pm);
    }
}

#endif // CANARD_DUMMY_CENTER_MAP_H

