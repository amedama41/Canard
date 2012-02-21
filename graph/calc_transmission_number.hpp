#ifndef CANARD_CALC_TRANSMISSION_NUMBER_H
#define CANARD_CALC_TRANSMISSION_NUMBER_H

#include <vector>
#include <numeric>
#include <functional>
#include <limits>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/property_map_iterator.hpp>
#include "Canard/graph/parallel_dijkstra_shortest_paths.hpp"

namespace Canard {
  // TODO Concept Check, NamedParam
  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map, IndexMap i_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      using boost::vertices;
      using boost::num_vertices;
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typedef typename std::vector<WeightValue> DistVector;
      typedef typename DistVector::iterator DistVecIter;
      typedef boost::iterator_property_map<
        DistVecIter, IndexMap,
        typename std::iterator_traits<DistVecIter>::value_type,
        typename std::iterator_traits<DistVecIter>::reference
        > DistanceMap;

      DistVector dist(num_vertices(g));
      DistanceMap d_map(dist.begin(), i_map);

      Canard::parallel_dijkstra_shortest_paths(
          g, center_map,
          boost::weight_map(w_map). distance_map(d_map).
          distance_compare(compare). distance_combine(combine).
          distance_inf(inf). distance_zero(zero));

      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      boost::tie(vi, vi_end) = vertices(g);

      return std::accumulate
        (boost::make_property_map_iterator(d_map, vi),
         boost::make_property_map_iterator(d_map, vi_end), zero, combine);
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map, IndexMap i_map)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;

      return calc_transmission_number
        (g, center_map, w_map, i_map,
         std::less<WeightValue>(), boost::closed_plus<WeightValue>(),
         std::numeric_limits<WeightValue>::max(), WeightValue());
    }

  template <class Graph, class CenterMap, class WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map)
    {
      return calc_transmission_number(g, center_map, w_map,
          get(boost::vertex_index, g));
    }
}

#endif  // CANARD_CALC_TRANSMISSION_NUMBER_H

