#ifndef CANARD_CALC_TRANSMISSION_NUMBER_H
#define CANARD_CALC_TRANSMISSION_NUMBER_H

#include <vector>
#include <functional>
#include <limits>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include "Canard/graph/parallel_dijkstra_shortest_paths.hpp"
#include "Canard/property_map/property_map_adaptor.hpp"

namespace Canard {
  // TODO Concept Check, NamedParam
  template <class Graph, class CenterMap, class WeightMap,
            class IndexMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map,
     IndexMap i_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      using boost::vertices;
      using boost::num_vertices;
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      typedef typename std::vector<DistValue> DistVector;
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

      boost::property_map_function<DemandMap> demand_func(demand_map);

      return boost::accumulate(
          vertices(g) | boost::adaptors::filtered(demand_func)
                      | Canard::adaptors::mapped(d_map),
          zero, combine);
    }

  template <class Graph, class CenterMap, class WeightMap,
            class IndexMap, class DemandMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map,
     IndexMap i_map, DemandMap demand_map)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;

      return calc_transmission_number(
          g, center_map, w_map, i_map, demand_map,
          std::less<WeightValue>(), boost::closed_plus<WeightValue>(),
          std::numeric_limits<WeightValue>::max(), WeightValue());
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return calc_transmission_number(
          g, center_map, w_map, i_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_transmission_number
    (const Graph& g, CenterMap center_map, WeightMap w_map)
    {
      return calc_transmission_number(
          g, center_map, w_map, get(boost::vertex_index, g));
    }
}

#endif  // CANARD_CALC_TRANSMISSION_NUMBER_H

