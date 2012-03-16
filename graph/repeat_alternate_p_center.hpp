#ifndef CANARD_REPEAT_ALTERNATE_P_CENTER_H
#define CANARD_REPEAT_ALTERNATE_P_CENTER_H

#include <vector>
#include <algorithm>
#include <limits>
#include <functional>
#include <stdexcept>
#include <boost/function.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include "Canard/graph/p_center_base.hpp"
#include "Canard/graph/random_p_center.hpp"
#include "Canard/graph/alternate_p_center.hpp"
#include "Canard/graph/calc_eccentricity.hpp"
#include "Canard/property_map/property_map_copy.hpp"
#include "Canard/property_map/property_map_adaptor.hpp"

namespace Canard {
  namespace detail {
    // Given center_map, and create a vector which containes center vertex
    template <class Graph, class CenterMap>
      const std::vector<typename boost::graph_traits<Graph>::vertex_descriptor>
      create_center_vector
      (const Graph& g, CenterMap center_map)
      {
        using boost::vertices;
        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;

        const std::pair<VIter, VIter> vrange(vertices(g));
        const boost::property_map_function<CenterMap> center_func(center_map);
        return std::vector<Vertex>(
            boost::make_filter_iterator(
              center_func, vrange.first,  vrange.second),
            boost::make_filter_iterator(
              center_func, vrange.second, vrange.second));
      }

    // Given a generator,
    // and create a vector which containes vertices belong to the generator
    template <class Graph, class Vertex, class GeneratorMap, class CandidateMap>
      const std::vector<Vertex>
      create_same_generator_candidates_vector
      (const Graph& g, const Vertex generator,
       GeneratorMap generator_map, CandidateMap candidate_map)
      {
        typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;

        const boost::property_map_function<GeneratorMap>
          generator_func(generator_map);
        const boost::property_map_function<CandidateMap>
          candidate_func(candidate_map);
        const boost::function<bool (Vertex)> is_same_generator_candidate(
            boost::lambda::bind(generator_func, boost::lambda::_1) == generator
            && boost::lambda::_1 != generator
            && boost::lambda::bind(candidate_func, boost::lambda::_1));

        const std::pair<VIter, VIter> vrange = vertices(g);
        return std::vector<Vertex>(
            boost::make_filter_iterator(
              is_same_generator_candidate, vrange.first,  vrange.second),
            boost::make_filter_iterator(
              is_same_generator_candidate, vrange.second, vrange.second));
      }

    // create a not empty allocable vertices vector
    template <class DistValue, class Graph, class Centers, class DistanceMatrix,
              class GeneratorMap, class MaxDistMap,
              class IndexMap, class CandidateMap, class DemandMap,
              class Compare, class DistZero>
      const std::vector<typename boost::graph_traits<Graph>::vertex_descriptor>
      create_candidates_vector
      (const Graph& g, Centers& centers, const DistanceMatrix& d_matrix,
       GeneratorMap generator_map, MaxDistMap max_dist_map,
       IndexMap i_map, CandidateMap candidate_map, DemandMap demand_map,
       Compare compare, DistZero zero)
      {
        using boost::vertices;
        using boost::num_vertices;

        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

        set_generator_and_max_dist<DistValue>(
            g, generator_map, max_dist_map,
            centers, d_matrix, i_map, demand_map, compare, zero);

        // sort centers in descending order
        const boost::property_map_function<MaxDistMap> max_dist_f(max_dist_map);
        boost::sort(
            centers,
            boost::lambda::bind(compare,
              boost::lambda::bind(max_dist_f, boost::lambda::_2),
              boost::lambda::bind(max_dist_f, boost::lambda::_1)));

        // create candidates vector
        for (typename Centers::iterator
            ci = centers.begin(), ci_end = centers.end(); ci != ci_end; ++ci) {
          const std::vector<Vertex> candidates(
              create_same_generator_candidates_vector(
                g, *ci, generator_map, candidate_map));
          if (!candidates.empty()) {
            return candidates;
          }
        }

        throw std::runtime_error("p is too large.");
      }

    // find next feasible allocated vertex
    template <class DistValue, class Graph, class CenterMap,
              class DistanceMatrix,
              class GeneratorMap, class MaxDistMap,
              class IndexMap, class CandidateMap, class DemandMap,
              class Compare, class DistInf, class DistZero>
      const typename boost::graph_traits<Graph>::vertex_descriptor
      get_next_vertex
      (const Graph& g, std::size_t times, CenterMap center_map,
       const DistanceMatrix& d_matrix,
       GeneratorMap generator_map, MaxDistMap max_dist_map,
       IndexMap i_map, CandidateMap candidate_map, DemandMap demand_map,
       Compare compare, DistInf inf, DistZero zero)
      {
        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

        std::vector<Vertex> centers(create_center_vector(g, center_map));

        std::vector<Vertex> candidates(
            create_candidates_vector<DistValue>(
              g, centers, d_matrix, generator_map, max_dist_map,
              i_map, candidate_map, demand_map, compare, zero));

        Vertex opt_vertex = candidates.front();
        DistValue opt_dist = inf;
        for (std::size_t t = 0; t < times && !candidates.empty(); ++t) {
          // TODO
          const std::size_t ind = std::rand() % candidates.size();
          const Vertex next_vertex = candidates[ind];
          put(center_map, next_vertex, 1);

          const DistValue dist(
              alternate_p_center<DistValue>(
                g, center_map, d_matrix, i_map,
                candidate_map, demand_map, compare, zero));
          if (compare(dist, opt_dist)) {
            opt_vertex = next_vertex;
            opt_dist = dist;
          }

          boost::fill(
              vertices(g) | Canard::adaptors::mapped(center_map),
              false);
          boost::fill(
              centers | Canard::adaptors::mapped(center_map),
              true);

          candidates[ind] = candidates.back();
          candidates.pop_back();
        }
        return opt_vertex;
      }
  } // namespace detail


  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    repeat_alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      if (p == 0) {
        throw std::runtime_error("Number of center is 0"__FILE__);
      }
      using boost::vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      typedef typename DistMatrix<DistValue>::type DistanceMatrix;

      // solve 1-center problem
      random_p_center(
          g, 1, 1, center_map, w_map, i_map,
          candidate_map, demand_map, compare, combine, inf, zero);
      const DistanceMatrix d_matrix(
          create_distance_matrix(g, w_map, i_map, compare, combine, inf, zero));
      alternate_p_center<DistValue>(
          g, center_map, d_matrix,
          i_map, candidate_map, demand_map, compare, zero);

      // solve (2 - p)-center problem
      std::vector<Vertex> generators(num_vertices(g));
      std::vector<DistValue> max_dists(num_vertices(g), zero);
      times = times / p + 1;
      for (std::size_t i = 1; i < p; ++i) {
        const Vertex opt_vertex(
            detail::get_next_vertex<DistValue>(
              g, times, center_map, d_matrix,
              boost::make_iterator_property_map(generators.begin(), i_map),
              boost::make_iterator_property_map(max_dists.begin(), i_map),
              i_map, candidate_map, demand_map, compare, inf, zero));
        put(center_map, opt_vertex, 1);
        alternate_p_center<DistValue>(
            g, center_map, d_matrix,
            i_map, candidate_map, demand_map, compare, zero);
      }
      return calc_eccentricity(
          g, center_map, w_map, i_map, demand_map, compare, combine, inf, zero);
    }


  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap>
    typename boost::property_traits<WeightMap>::value_type
    repeat_alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map)
    {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      return repeat_alternate_p_center(
          g, p, times, center_map, w_map, i_map, candidate_map, demand_map,
          std::less<DistValue>(), boost::closed_plus<DistValue>(),
          std::numeric_limits<DistValue>::max(), DistValue());
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap>
    typename boost::property_traits<WeightMap>::value_type
    repeat_alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map, CandidateMap candidate_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return repeat_alternate_p_center(
          g, p, times, center_map, w_map, i_map, candidate_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    typename boost::property_traits<WeightMap>::value_type
    repeat_alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return repeat_alternate_p_center(
          g, p, times, center_map, w_map, i_map,
          boost::make_constant_property<Vertex>(true));
    }
} // namespace Canard

#endif  // CANARD_REPEAT_ALTERNATE_P_CENTER_H

