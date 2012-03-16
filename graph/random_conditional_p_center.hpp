#ifndef CANARD_RANDOM_CONDITIONAL_P_CENTER_H
#define CANARD_RANDOM_CONDITIONAL_P_CENTER_H

#include "Canard/graph/random_p_center.hpp"
#include "Canard/graph/conditional_p_center.hpp"

namespace Canard {
  namespace detail {
    template <class WeightMap, class IndexMap,
              class Compare, class Combine, class DistInf, class DistZero>
      struct RandomPCenter {
        RandomPCenter(std::size_t times, WeightMap w, IndexMap i,
            Compare cmp, Combine cmb, DistInf inf, DistZero zero)
          : times(times), w_map(w), i_map(i),
            compare(cmp), combine(cmb), inf(inf), zero(zero)
        { }
        template <class Graph, class CenterMap,
                  class CandidateMap, class DemandMap>
          typename boost::property_traits<WeightMap>::value_type
          operator()(const Graph& g, std::size_t p, CenterMap center_map, 
              CandidateMap candidate_map, DemandMap demand_map) const
          {
            return random_p_center(g, p, times, center_map, w_map, i_map,
                candidate_map, demand_map, compare, combine, inf, zero);
          }
        std::size_t times;
        WeightMap w_map;
        IndexMap i_map;
        Compare compare;
        Combine combine;
        DistInf inf;
        DistZero zero;
      };
    template <class WeightMap, class IndexMap,
              class Compare, class Combine, class DistInf, class DistZero>
      inline RandomPCenter<
        WeightMap, IndexMap, Compare, Combine, DistInf, DistZero
      >
      make_random_p_center_func
      (std::size_t times, WeightMap w, IndexMap i,
       Compare cmp, Combine cmb, DistInf inf, DistZero zero)
      {
        return RandomPCenter<
            WeightMap, IndexMap, Compare, Combine, DistInf, DistZero
          >(times, w, i, cmp, cmb, inf, zero);
      }
  } // namespace detail

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    random_conditional_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      return conditional_p_center(g, p, center_map,
          detail::make_random_p_center_func(times, w_map, i_map,
            compare, combine, inf, zero),
          existed_center_map, w_map, i_map, candidate_map, demand_map,
          compare, combine, inf, zero);
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap>
    inline typename boost::property_traits<WeightMap>::value_type
    random_conditional_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map)
    {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      return random_conditional_p_center(g, p, times, center_map,
          existed_center_map, w_map, i_map, candidate_map, demand_map,
          std::less<DistValue>(), boost::closed_plus<DistValue>(),
          std::numeric_limits<DistValue>::max(), DistValue());
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap, class CandidateMap>
    inline typename boost::property_traits<WeightMap>::value_type
    random_conditional_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     ExistedCenterMap existed_center_map,
     WeightMap w_map, IndexMap i_map, CandidateMap candidate_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return random_conditional_p_center(g, p, times, center_map,
          existed_center_map, w_map, i_map, candidate_map,
          boost::make_constant_property<Vertex>(1));
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap, class IndexMap>
    inline typename boost::property_traits<WeightMap>::value_type
    random_conditional_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     ExistedCenterMap existed_center_map, WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return random_conditional_p_center(g, p, times, center_map,
          existed_center_map, w_map, i_map,
          boost::make_constant_property<Vertex>(1));
    }

  template <class Graph, class CenterMap, class ExistedCenterMap,
            class WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    random_conditional_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     ExistedCenterMap existed_center_map, WeightMap w_map)
    {
      return random_conditional_p_center(g, p, times, center_map,
          existed_center_map, w_map, get(boost::vertex_index, g));
    }
}

#endif // CANARD_RANDOM_CONDITIONAL_P_CENTER_H

