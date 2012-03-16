#ifndef CANARD_SPLIT_LINEAR_P_CENTER_H
#define CANARD_SPLIT_LINEAR_P_CENTER_H

#include <vector>
#include <queue>
#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include "Canard/graph/calc_eccentricity.hpp"
#include "Canard/graph/split_to_linear_graphs.hpp"
#include "Canard/graph/glpk_p_center.hpp"
#include "Canard/graph/linear_p_center_graph_info.hpp"
#include "Canard/graph/indirected_less.hpp"
#include "Canard/graph/dummy_center_map.hpp"

namespace Canard {
  template <class Graph, class CenterMap, class WeightMap, class EdgeColorMap,
            class IndexMap, class ExistedCenterMap, class CandidateMap>
    double
    split_linear_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     WeightMap w_map, EdgeColorMap color_map, IndexMap i_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      typedef linear_graphs<Graph> LinearGraphs;
      typedef typename LinearGraphs::value_type LinearGraph;
      typedef detail::linear_pcenter_graph_info<LinearGraph> LinearGInfo;

      LinearGraphs lgraphs
        = split_to_linear_graphs(g, color_map, existed_center_map);

      // Guarantee that not 2 degree vertices are exisited centers.
      dummy_center_map<Graph, ExistedCenterMap>
        dum_center_map(g, existed_center_map);

      std::vector<LinearGInfo> infos;
      infos.reserve(lgraphs.size());

      std::priority_queue<
        LinearGInfo*, std::vector<LinearGInfo*>, indirected_less<LinearGInfo>
      > que;

      for (typename LinearGraphs::iterator
          it = lgraphs.begin(); it != lgraphs.end(); ++it) {
        infos.push_back(
            LinearGInfo(*it, dum_center_map, candidate_map, w_map, i_map));
        if (infos.back().num_alloc_pos > 0) {
          que.push(&(infos.back()));
        }
      }

      for (std::size_t i = 0; i < p; ++i) {
        if (que.empty()) {
          throw std::runtime_error("The number of center is too large");
        }
        LinearGInfo& info = *(que.top());
        que.pop();
        ++info.num_center;
        info.cover_dist
          = Canard::glpk_p_center(*(info.g), info.num_center, center_map,
              w_map, i_map, dum_center_map, candidate_map);
        if (info.num_center < info.num_alloc_pos) {
          que.push(&info);
        }
      }

      return (*std::max_element(infos.begin(), infos.end())).cover_dist;
    }

  template <class Graph, class CenterMap, class WeightMap,
            class EdgeColorMap, class IndexMap, class ExistedCenterMap>
    inline double
    split_linear_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     WeightMap w_map, EdgeColorMap color_map,
     IndexMap i_map, ExistedCenterMap existed_center_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return split_linear_p_center(g, p, center_map,
          w_map, color_map, i_map, existed_center_map,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap,
            class EdgeColorMap, class IndexMap>
    inline double
    split_linear_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     WeightMap w_map, EdgeColorMap color_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return split_linear_p_center(g, p, center_map,
          w_map, color_map, i_map,
          boost::make_constant_property<Vertex>(false));
    }

  template <class Graph, class CenterMap, class WeightMap, class EdgeColorMap>
    inline double
    split_linear_p_center
    (const Graph& g, std::size_t p, CenterMap center_map,
     WeightMap w_map, EdgeColorMap color_map)
    {
      return split_linear_p_center(g, p, center_map,
          w_map, color_map, get(boost::vertex_index, g));
    }
}

#endif  // CANARD_SPLIT_LINEAR_P_CENTER_H

