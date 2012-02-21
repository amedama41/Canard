#ifndef CANARD_UNEVEN_DIST_CENTER_HPP
#define CANARD_UNEVEN_DIST_CENTER_HPP

#include <vector>
#include <queue>
#include <numeric>
#include <functional>
#include <boost/ref.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include "Canard/graph/split_to_linear_graphs.hpp"
#include "Canard/graph/convert_linear_graph_to_acyclic_graph.hpp"
#include "Canard/graph/uneven_dist_graph_info.hpp"
#include "Canard/graph/indirected_less.hpp"
#include "Canard/property_map/property_map_copy.hpp"

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
      typedef LinearGraphs<Graph> LinearGraphs;
      typedef typename LinearGraphs::value_type LinearGraph;
      typedef typename AcyclicGraph<LinearGraph, WeightMap>::type AcyclicGraph;
      typedef
        typename boost::property_map<AcyclicGraph, boost::edge_bundle_t>::type
        AcyclicWeightMap;
      typedef
        detail::GraphInfo<AcyclicGraph, AcyclicWeightMap, Compare, Combine>
        GraphInfo;
      typedef std::vector<GraphInfo> GraphInfos;

      const LinearGraphs linear_graphs
        = split_to_linear_graphs(g, color_map, existed_center_map);

      /* TODO
      if (std::accumulate(linear_graphs.begin(), linear_graphs.end(), 0) < k) {
        throw std::runtime_error("The number of center is too large");
      }
      */

      // convert linear graph to acyclic graph
      GraphInfos graph_infos;
      graph_infos.reserve(linear_graphs.size());
      typename GraphInfo::OptSols optimal_solutions;
      typename GraphInfo::OptResConts optimal_resource_containers;
      std::priority_queue<
        GraphInfo*, std::vector<GraphInfo*>, indirected_less<GraphInfo>
      > que;
      for (typename LinearGraphs::const_iterator it = linear_graphs.begin();
          it != linear_graphs.end(); ++it) {
        graph_infos.push_back(
            GraphInfo(
              convert_linear_graph_to_acyclic_graph(
                *it, weight_map, candidate_map, combine, multiple),
              compare, combine, zero,
              optimal_solutions, optimal_resource_containers));
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
      std::for_each(vi, vi_end, make_property_map_copy(
            center_map, boost::make_constant_property<Vertex>(false)));
      for (typename GraphInfos::const_iterator it = graph_infos.begin();
          it != graph_infos.end(); ++it) {
        const AcyclicGraph& graph = it->graph;
        typename GraphInfo::EdgeVector::const_iterator
          first = it->prev_solution.begin(), last = it->prev_solution.end();
        put(center_map, graph[target(*first, graph)], true);
        for (; first != last; ++first) {
          put(center_map, graph[source(*first, graph)], true);
        }
      }

      return std::accumulate(
          boost::make_transform_iterator(
            graph_infos.begin(), std::mem_fun_ref(&GraphInfo::get_value)),
          boost::make_transform_iterator(
            graph_infos.end(),   std::mem_fun_ref(&GraphInfo::get_value)), 0);
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
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return uneven_dist_center(
          g, k, center_map, weight_map, color_map,
          existed_center_map, candidate_map,
          std::less<WeightValue>(),
          boost::closed_plus<WeightValue>(),
          std::multiplies<WeightValue>(),
          WeightValue());
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

