#ifndef CANARD_RANDOM_P_CENETER_H
#define CANARD_RANDOM_P_CENETER_H

#include <vector>
#include <limits>
#include <functional>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/algorithm/fill.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>
#include "Canard/graph/calc_eccentricity.hpp"
#include "Canard/property_map/property_map_copy.hpp"
#include "Canard/property_map/property_map_adaptor.hpp"

namespace Canard {
  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    random_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      using boost::vertices;
      using boost::num_vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;

      typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
      const std::pair<VIter, VIter> vrange = vertices(g);
      boost::property_map_function<CandidateMap> cand_map_func(candidate_map);
      std::vector<Vertex> candidates(
          boost::make_filter_iterator(cand_map_func, vrange.first, vrange.second),
          boost::make_filter_iterator(cand_map_func, vrange.second, vrange.second));
      if (candidates.size() < p) {
        throw std::runtime_error("Number of center is too large."__FILE__);
      }

      std::vector<char> temp_center(num_vertices(g), false);
      boost::iterator_property_map<std::vector<char>::iterator, IndexMap>
        temp_center_map(temp_center.begin(), i_map);

      DistValue opt_dist = inf;
      for (std::size_t i = 0; i < times; ++i) {
        // TODO Random generator
        boost::random_shuffle(candidates);
        // Set temp_center_map
        boost::fill(
            candidates | boost::adaptors::sliced(0, p)
                       | Canard::adaptors::mapped(temp_center_map),
            true);

        const DistValue max_dist = calc_eccentricity(g, temp_center_map,
            w_map, i_map, demand_map, compare, combine, inf, zero);
        if (compare(max_dist, opt_dist)) {
          boost::for_each(vrange,
              make_property_map_copy(center_map, temp_center_map));
          opt_dist = max_dist;
        }

        // Reset temp_center_map
        boost::fill(
            candidates | boost::adaptors::sliced(0, p)
                       | Canard::adaptors::mapped(temp_center_map),
            true);
      }

      return opt_dist;
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap>
    typename boost::property_traits<WeightMap>::value_type
    random_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
      CandidateMap candidate_map, DemandMap demand_map)
    {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      return random_p_center(
          g, p, times, center_map, w_map, i_map,
          candidate_map, demand_map,
          std::less<DistValue>(), boost::closed_plus<DistValue>(),
          std::numeric_limits<DistValue>::max(), DistValue());
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap>
    typename boost::property_traits<WeightMap>::value_type
    random_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return random_p_center(
          g, p, times, center_map, w_map, i_map,
          candidate_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    typename boost::property_traits<WeightMap>::value_type
    random_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return random_p_center(
          g, p, times, center_map, w_map, i_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap>
    typename boost::property_traits<WeightMap>::value_type
    random_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map)
    {
      return random_p_center(
          g, p, times, center_map, w_map, get(boost::vertex_index, g));
    }
} // namespace Canard

#endif  // CANARD_RANDOM_P_CENETER_H

