#ifndef CANARD_GREEDY_UNEVEN_DIST_CENTER_H
#define CANARD_GREEDY_UNEVEN_DIST_CENTER_H

#include <list>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/adaptors.hpp>
#include "Canard/graph/calc_uneven_distribution.hpp"
#include "Canard/property_map/property_map_copy.hpp"

namespace Canard {
  namespace detail {
    template <class Graph, class Candidate,
              class CenterMap, class WeightMap, class IndexMap>
      double set_next_optimal_vertex
      (const Graph& g, Candidate& candidate,
       CenterMap center_map, WeightMap weight_map, IndexMap index_map)
      {
        typedef typename Candidate::iterator CandIter;
        CandIter opt_vertex_i = candidate.end();
        double opt_udist = std::numeric_limits<double>::max();
        for (CandIter ci = candidate.begin(), ci_end = candidate.end();
            ci != ci_end; ++ci) {
          put(center_map, *ci, 1);
          const double udist = calc_uneven_distribution
            (g, center_map, weight_map, boost::vertex_index_map(index_map));
          if (udist < opt_udist) {
            opt_vertex_i = ci;
            opt_udist = udist;
          }
          put(center_map, *ci, 0);
        }
        if (opt_vertex_i == candidate.end()) {
          throw std::runtime_error("number of center is too large");
        }
        put(center_map, *opt_vertex_i, 1);
        candidate.erase(opt_vertex_i);
        return opt_udist;
      }
  }

  template <class Graph, class CenterMap, class WeightMap, class VertexIndexMap,
            class ExistedCenterMap, class CandidateMap>
    double greedy_uneven_dist_center_impl
    (const Graph& g, std::size_t k,
     CenterMap center_map, WeightMap weight_map, VertexIndexMap index_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      using boost::vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef std::list<Vertex> Candidate;

      boost::property_map_function<CandidateMap> cand_map_func(candidate_map);
      typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
      const std::pair<VIter, VIter> vi = vertices(g);
      Candidate candidate(
          boost::make_filter_iterator(cand_map_func, vi.first,  vi.second),
          boost::make_filter_iterator(cand_map_func, vi.second, vi.second));

      boost::for_each(vi,
          make_property_map_copy(center_map, existed_center_map));
      typename Candidate::iterator start_vertex = candidate.begin();
      // TODO
      put(center_map, *start_vertex, 1);
      candidate.erase(start_vertex);

      double udist = std::numeric_limits<double>::max();
      for (std::size_t i = 1; i < k; ++i) {
        udist = detail::set_next_optimal_vertex(
            g, candidate, center_map, weight_map, index_map);
      }

      boost::property_map_function<ExistedCenterMap>
        existedc_map_func(existed_center_map);
      boost::for_each(vi | boost::adaptors::filtered(existedc_map_func),
          make_property_map_copy(
            center_map, boost::make_constant_property<Vertex>(0)));

      return udist;
    }

  template <class Graph, class CenterMap, class WeightMap, class VertexIndexMap,
            class ExistedCenterMap, class CandidateMap>
    double greedy_uneven_dist_center
    (const Graph& g, std::size_t k, std::size_t times,
     CenterMap center_map, WeightMap weight_map, VertexIndexMap index_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      using boost::vertices;
      using boost::num_vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      std::vector<Vertex> temp_center(num_vertices(g), 0);
      boost::iterator_property_map<
        typename std::vector<Vertex>::iterator, VertexIndexMap
        > temp_center_map(temp_center.begin(), index_map);
      double opt_udist = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < times; ++i) {
        const double udist = greedy_uneven_dist_center_impl
          (g, k, temp_center_map, weight_map, index_map,
           existed_center_map, candidate_map);
        if (udist < opt_udist) {
          boost::for_each(vertices(g),
              make_property_map_copy(center_map, temp_center_map));
          opt_udist = udist;
        }
      }
      return opt_udist;
    }

  template <class Graph, class CenterMap,
            class WeightMap, class VertexIndexMap,
            class ExistedCenterMap>
    double greedy_uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, VertexIndexMap index_map,
     ExistedCenterMap existed_center_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return greedy_uneven_dist_center(g, k, center_map, weight_map, index_map,
          existed_center_map, boost::make_constant_property<Vertex>(1));
    }

  template <class Graph, class CenterMap, class WeightMap, class VertexIndexMap>
    double greedy_uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, VertexIndexMap index_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return greedy_uneven_dist_center(g, k, center_map, weight_map, index_map,
          boost::make_constant_property<Vertex>(0));
    }

  template <class Graph, class CenterMap, class WeightMap>
    double greedy_uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map, WeightMap weight_map)
    {
      return greedy_uneven_dist_center(g, k,
          center_map, weight_map, get(boost::vertex_index, g));
    }
}

#endif  // CANARD_GREEDY_UNEVEN_DIST_CENTER_H

