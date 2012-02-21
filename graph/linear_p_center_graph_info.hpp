#ifndef CANARD_LINEAR_P_CENTER_GRAPH_INFO_H
#define CANARD_LINEAR_P_CENTER_GRAPH_INFO_H

#include <boost/graph/graph_traits.hpp>

namespace Canard {
  namespace detail {
    template <class Graph>
      struct LinearPCenterGraphInfo {
        const Graph* g;
        double cover_dist;
        std::size_t num_center;
        std::size_t num_alloc_pos;

        template <class ExistedCenterMap, class CandidateMap,
                  class WeightMap, class IndexMap>
          LinearPCenterGraphInfo
          (const Graph& g,
           ExistedCenterMap e_map, CandidateMap c_map,
           WeightMap w_map, IndexMap i_map)
          : g(&g),
            cover_dist(calc_eccentricity(g, e_map, w_map, i_map)),
            num_center(0),
            num_alloc_pos(0)
          {
            using boost::vertices;
            using boost::out_degree;
            typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
            for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
              if (get(c_map, *vi) != 0 &&
                  get(e_map, *vi) == 0 &&
                  out_degree(*vi, g) == 2) {
                ++num_alloc_pos;
              }
            }
          }

        bool operator<(const LinearPCenterGraphInfo& rhs) const {
          if (cover_dist == rhs.cover_dist) {
            return num_center < rhs.num_center;
          }
          return cover_dist < rhs.cover_dist;
        }
      };
  }
}

#endif // CANARD_LINEAR_P_CENTER_GRAPH_INFO_H

