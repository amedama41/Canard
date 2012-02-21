#ifndef CANARD_CONDITIONAL_P_CENTER_H
#define CANARD_CONDITIONAL_P_CENTER_H

#include <vector>
#include <algorithm>
#include <functional>
#include <limits>
#include <boost/function.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include "Canard/graph/parallel_dijkstra_shortest_paths.hpp"
#include "Canard/property_map/logical_property_map.hpp"

namespace Canard {
  namespace detail {
    template <class DistanceMap, class Compare>
      struct DistGreater {
        typedef bool result_type;
        typedef typename DistanceMap::key_type Vertex;
        DistGreater(DistanceMap d_map, Compare compare)
          : d_map(d_map), compare(compare) { }
        result_type operator()(Vertex lhs, Vertex rhs) const
        {
          return compare(get(d_map, rhs), get(d_map, lhs));
        }
        DistanceMap d_map;
        Compare compare;
      };
    template <class DistanceMap, class Compare>
      inline DistGreater<DistanceMap, Compare>
      make_dist_greater(DistanceMap d_map, Compare cmp)
      { return DistGreater<DistanceMap, Compare>(d_map, cmp); }

    template <class DemandMap, class DemandVec>
      inline void set_demand_map
      (DemandMap d_map, const DemandVec& demands, std::size_t r)
      {
        const typename DemandVec::const_iterator
          demand_i = demands.begin(), demand_end = demands.end();
        std::fill(
            boost::make_property_map_iterator(d_map, demand_i),
            boost::make_property_map_iterator(d_map, demand_i + r), 1);
        std::fill(
            boost::make_property_map_iterator(d_map, demand_i + r),
            boost::make_property_map_iterator(d_map, demand_end), 0);
      }
  } // detail

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap,
            class IndexMap
              = typename boost::property_map<Graph, boost::vertex_index_t>::type,
            class CandidateMap
              = boost::constant_property_map<
                typename boost::graph_traits<Graph>::vertex_descriptor, bool> >
    struct PCenterFunc {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      typedef not_predict_map<ExistedCenterMap> NotPredictMap;
      typedef and_predict_map<NotPredictMap, CandidateMap> CMap;
      typedef std::vector<char> Demands;
      typedef boost::iterator_property_map<Demands::iterator, IndexMap> DMap;
      typedef boost::function<
        DistValue (const Graph&, std::size_t, CenterMap, CMap, DMap)
      > type;
    };

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    conditional_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     typename PCenterFunc<
       Graph, CenterMap, ExistedCenterMap, WeightMap, IndexMap, CandidateMap
     >::type p_center_func,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      using boost::vertices;
      using boost::num_vertices;

      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
      typedef std::vector<DistValue> DistVec;
      typedef std::vector<char> DemandVec;

      DistVec distance(num_vertices(g), zero);
      boost::iterator_property_map<typename DistVec::iterator, IndexMap>
        d_map(distance.begin(), i_map);

      parallel_dijkstra_shortest_paths(g, existed_center_map,
          boost::weight_map(w_map). vertex_index_map(i_map).
          distance_map(d_map).  distance_compare(compare).
          distance_combine(combine).  distance_inf(inf). distance_zero(zero));

      const std::pair<VIter, VIter> vi = vertices(g);
      boost::property_map_function<DemandMap> demand_func(demand_map);
      std::vector<Vertex> demand_vertices(
          boost::make_filter_iterator(demand_func, vi.first,  vi.second),
          boost::make_filter_iterator(demand_func, vi.second, vi.second));
      std::sort(demand_vertices.begin(), demand_vertices.end(),
          detail::make_dist_greater(d_map, compare));

      DemandVec conditional_demand(num_vertices(g), 0);
      boost::iterator_property_map<DemandVec::iterator, IndexMap>
        conditional_demand_map(conditional_demand.begin(), i_map);

      std::size_t max_r = demand_vertices.size() - 1;
      std::size_t min_r = std::min(p, max_r);
      while (max_r != min_r) {
        const std::size_t r = (max_r + min_r) / 2;
        detail::set_demand_map(conditional_demand_map, demand_vertices, r);

        // Graph, Number of center, CenterMap, CandidateMap, DemandMap
        const DistValue dist = p_center_func(g, p, center_map,
            make_and_predict_map(
              make_not_predict_map(existed_center_map), candidate_map),
            conditional_demand_map);
        if (compare(get(d_map, demand_vertices[r]), dist)) {
          max_r = r;
        }
        else {
          min_r = r;
        }
      } // while

      detail::set_demand_map(conditional_demand_map, demand_vertices, min_r);
      return p_center_func(g, p, center_map,
          make_and_predict_map(
            make_not_predict_map(existed_center_map), candidate_map),
          conditional_demand_map);
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap>
    inline typename boost::property_traits<WeightMap>::value_type
    conditional_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     typename PCenterFunc<
       Graph, CenterMap, ExistedCenterMap, WeightMap, IndexMap, CandidateMap
     >::type p_center_func,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map)
    {
      typedef typename boost::property_traits<WeightMap> DistValue;
      return conditional_p_center(g, p, center_map, p_center_func,
          existed_center_map, w_map, i_map, candidate_map, demand_map,
          std::less<DistValue>(), boost::closed_plus<DistValue>(),
          std::numeric_limits<DistValue>::max(), DistValue());
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap, class CandidateMap>
    inline typename boost::property_traits<WeightMap>::value_type
    conditional_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     typename PCenterFunc<
       Graph, CenterMap, ExistedCenterMap, WeightMap, IndexMap, CandidateMap
     >::type p_center_func,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map, CandidateMap candidate_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return conditional_p_center(g, p, center_map, p_center_func,
          existed_center_map, w_map, i_map, candidate_map,
          boost::make_constant_property<Vertex>(1));
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap>
    inline typename boost::property_traits<WeightMap>::value_type
    conditional_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     typename PCenterFunc<
       Graph, CenterMap, ExistedCenterMap, WeightMap, IndexMap
     >::type p_center_func,
     ExistedCenterMap existed_center_map, WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return conditional_p_center(g, p, center_map, p_center_func,
          existed_center_map, w_map, i_map,
          boost::constant_property_map<Vertex, bool>(true));
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    conditional_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     typename PCenterFunc<Graph, CenterMap, ExistedCenterMap, WeightMap>::type
     p_center_func,
     ExistedCenterMap existed_center_map, WeightMap w_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return conditional_p_center(g, p, center_map, p_center_func,
          existed_center_map, w_map, get(boost::vertex_index, g));
    }
} // Canard

#endif  // CANARD_CONDITIONAL_P_CENTER_H

