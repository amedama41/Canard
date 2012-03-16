#ifndef CANARD_UNEVEN_DIST_CENTER_HPP
#define CANARD_UNEVEN_DIST_CENTER_HPP

#include <vector>
#include <queue>
#include <functional>
#include <boost/ref.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/numeric.hpp>
#include "Canard/graph/split_to_linear_graphs.hpp"
#include "Canard/graph/convert_linear_graph_to_acyclic_graph.hpp"
#include "Canard/graph/uneven_dist_graph_info.hpp"
#include "Canard/graph/indirected_less.hpp"
#include "Canard/property_map/property_map_copy.hpp"
#include "Canard/property_map/property_map_adaptor.hpp"

namespace Canard {
  template <class Graph, class CenterMap, class WeightMap, class EdgeColorMap,
            class ExistedCenterMap, class CandidateMap,
            class Compare, class Combine, class Multiple, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, EdgeColorMap color_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map,
     Compare compare, Combine combine, Multiple multiple, DistZero zero)
    {
      using boost::vertices;
      using boost::source;
      using boost::target;
      typedef linear_graphs<Graph> LinearGraphs;
      typedef typename LinearGraphs::value_type LinearGraph;
      typedef typename AcyclicGraph<LinearGraph, WeightMap>::type AcyclicGraph;
      typedef
        typename boost::property_map<AcyclicGraph, boost::edge_bundle_t>::type
        AcyclicWeightMap;
      typedef
        detail::graph_info<AcyclicGraph, AcyclicWeightMap, Compare, Combine>
        GraphInfo;
      typedef std::vector<GraphInfo> GraphInfos;

      const LinearGraphs lgraphs
        = split_to_linear_graphs(g, color_map, existed_center_map);

      /* TODO
      if (std::accumulate(lgraphs.begin(), lgraphs.end(), 0) < k) {
        throw std::runtime_error("The number of center is too large");
      }
      */

      // convert linear graph to acyclic graph
      GraphInfos graph_infos;
      graph_infos.reserve(lgraphs.size());
      typename GraphInfo::OptimalSolutions optimal_solutions;
      typename GraphInfo::OptimalResouceContainers optimal_resource_containers;
      std::priority_queue<
          GraphInfo*, std::vector<GraphInfo*>, indirected_less<GraphInfo>
        > que;
      for (typename LinearGraphs::const_iterator
          it = lgraphs.begin(); it != lgraphs.end(); ++it) {
#ifndef __GXX_EXPERIMENTAL_CXX0X__
        graph_infos.push_back(
            GraphInfo(
              convert_linear_graph_to_acyclic_graph(
                *it, weight_map, candidate_map, combine, multiple),
              compare, combine, zero,
              optimal_solutions, optimal_resource_containers));
#else
        graph_infos.emplace_back(
            convert_linear_graph_to_acyclic_graph(
              *it, weight_map, candidate_map, combine, multiple),
            compare, combine, zero,
            optimal_solutions, optimal_resource_containers);
#endif
        if (graph_infos.back().is_allocable()) {
          que.push(&graph_infos.back());
        }
      }

      // allocate center number
      for (std::size_t i = 0; i < k; ++i) {
        if (que.empty()) {
          throw std::runtime_error("The number of center is too large");
        }
        GraphInfo& info = *(que.top());
        que.pop();
        info.update();
        if (info.is_allocable()) {
          que.push(&info);
        }
      }

      // set center_map
      // TODO Exclude existed center?
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      boost::fill(
          vertices(g) | Canard::adaptors::mapped(center_map), false);
      for (typename GraphInfos::const_iterator
          it = graph_infos.begin(); it != graph_infos.end(); ++it) {
        const AcyclicGraph& graph = it->graph;
        typename GraphInfo::EdgeVector::const_iterator
          first = it->prev_solution.begin(), last = it->prev_solution.end();
        put(center_map, graph[target(*first, graph)], true);
        for (; first != last; ++first) {
          put(center_map, graph[source(*first, graph)], true);
        }
      }

      return boost::accumulate(
          graph_infos | boost::adaptors::transformed(
            std::mem_fun_ref(&GraphInfo::get_value)),
          zero);
    }

  // Overload ==============================================================
  template <class Graph, class CenterMap, class WeightMap, class EdgeColorMap,
            class ExistedCenterMap, class CandidateMap>
    typename boost::property_traits<WeightMap>::value_type
    uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, EdgeColorMap color_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      return uneven_dist_center(
          g, k, center_map, weight_map, color_map,
          existed_center_map, candidate_map,
          std::less<DistValue>(),
          boost::closed_plus<DistValue>(),
          std::multiplies<DistValue>(),
          DistValue());
    }

  template <class Graph, class CenterMap, class WeightMap,
            class EdgeColorMap, class ExistedCenterMap>
    typename boost::property_traits<WeightMap>::value_type
    uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, EdgeColorMap color_map,
     ExistedCenterMap existed_center_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return uneven_dist_center(
          g, k, center_map, weight_map, color_map,
          existed_center_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap, class EdgeColorMap>
    typename boost::property_traits<WeightMap>::value_type
    uneven_dist_center
    (const Graph& g, std::size_t k, CenterMap center_map,
     WeightMap weight_map, EdgeColorMap color_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return uneven_dist_center(
          g, k, center_map, weight_map, color_map,
          boost::make_constant_property<Vertex>(false));
    }
}

#endif  // CANARD_UNEVEN_DIST_CENTER_HPP

