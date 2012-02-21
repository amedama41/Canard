#ifndef CANARD_P_CENTER_BASE_H
#define CANARD_P_CENTER_BASE_H

#include <limits>
#include <algorithm>
#include <functional>
#include <boost/multi_array.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/relax.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/lambda/lambda.hpp>

namespace Canard {
  template <class WeightValue>
    struct DistMatrix {
      typedef boost::multi_array<WeightValue, 2> type;
    };

  template <class Graph, class WeightMap, class IndexMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename DistMatrix<
      typename boost::property_traits<WeightMap>::value_type
    >::type
    create_distance_matrix
    (const Graph& g, WeightMap w_map, IndexMap i_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      using boost::num_vertices;
      const typename boost::graph_traits<Graph>::vertices_size_type
        num_v = num_vertices(g);

      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typedef boost::multi_array<WeightValue, 2> DistanceMatrix;
      DistanceMatrix d_matrix(boost::extents[num_v][num_v]);

      boost::johnson_all_pairs_shortest_paths
        (g, d_matrix,
         boost::weight_map(w_map). vertex_index_map(i_map).
         distance_compare(compare). distance_combine(combine).
         distance_inf(inf). distance_zero(zero));

      return d_matrix;
    }

  template <class Graph, class WeightMap, class IndexMap>
    inline typename DistMatrix<
      typename boost::property_traits<WeightMap>::value_type
    >::type
    create_distance_matrix
    (const Graph& g, WeightMap w_map, IndexMap i_map)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return create_distance_matrix(g, w_map, i_map,
          std::less<WeightValue>(), boost::closed_plus<WeightValue>(),
          std::numeric_limits<WeightValue>::max(), WeightValue());
    }

  template <class WeightValue, class Graph, class DistanceMatrix,
            class IndexMap, class ExistedCenterMap>
    void set_distance_matrix
    (const Graph& g, DistanceMatrix& d_matrix,
     IndexMap i_map, ExistedCenterMap e_c_map)
    {
      using boost::vertices;
      typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
      VIter vi, vi_end;
      for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if (get(e_c_map, *vi) != false) {
          VIter ui, ui_end;
          for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
            const std::size_t i = get(i_map, *ui);
            const WeightValue d = d_matrix[i][get(i_map, *vi)];
            std::replace_if(d_matrix[i].begin(), d_matrix[i].end(),
                d < boost::lambda::_1, d);
          }
        }
      }
    }

  template <class Graph, class Vertices,
            class ExistedCenterMap, class CandidateMap>
    void set_candidates_and_demands
    (const Graph& g, Vertices& candidates, Vertices& demands,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      using boost::vertices;
      const typename boost::graph_traits<Graph>::vertices_size_type
        num_v = num_vertices(g);

      candidates.reserve(num_v);
      demands.reserve(num_v);

      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if (get(candidate_map, *vi) != false &&
            get(existed_center_map, *vi) == false) {
          candidates.push_back(*vi);
        }
        demands.push_back(*vi);
      }

      // TODO copy_if or filter_iterator
      //
      if (candidates.empty()) {
        throw std::runtime_error("The number of candidate vertex is too small");
      }
    }
}

#endif  // CANARD_P_CENTER_BASE_H

