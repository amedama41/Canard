#ifndef CANARD_GLPK_P_CENTER_H
#define CANARD_GLPK_P_CENTER_H

#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/algorithm/fill.hpp>
#include <glpk.h>
#include "Canard/graph/p_center_base.hpp"
#include "Canard/property_map/property_map_copy.hpp"
#include "Canard/property_map/property_map_adaptor.hpp"

namespace Canard {
  template <class DistanceMatrix, class IndexMap, class Vertex>
    boost::shared_ptr<glp_prob>
    make_p_center_prob
    (std::size_t p, const DistanceMatrix& d_matrix, IndexMap i_map,
     std::vector<Vertex>& candidates, std::vector<Vertex>& demands)
    {
      // create p-center problem ===================================
      boost::shared_ptr<glp_prob> g_prob(glp_create_prob(), glp_delete_prob);
      glp_prob *gp = g_prob.get();
      // glp_set_prob_name(gp, "p-Center");
      // glp_set_obj_name(gp, "Cover dist");
      glp_set_obj_dir(gp, GLP_MIN);

      const std::size_t num_demand = demands.size();
      const std::size_t num_x = candidates.size();
      const std::size_t num_y = num_x * num_demand;

      // x_1,x_2,...,x_n,y_11,y_12,...,y_1n,y_21,...,y_mn,z
      glp_add_cols(gp, num_x + num_y + 1);
      glp_add_rows(gp,
          1                     // #1 sumation of x_1 p
          + num_demand          // #2 sumation of y_ji for each i = 1
          + num_x * num_demand  // #3 y_ji - x_i <= 0
          + num_demand          // #4 sumation (y_ji * d_ji) for i - z <= 0
      );

      // 0 <= x_i <= 1, 0 <= y_ji <= 1
      std::size_t col_num = 1;
      for (std::size_t i = 1; i <= num_x + num_y; ++i) {
        glp_set_col_kind(gp, col_num, GLP_BV);
        glp_set_col_bnds(gp, col_num++, GLP_DB, 0.0, 1.0);
      }

      // z is cover dist
      // glp_set_col_name(gp, col_num, "Cover dist");
      glp_set_col_bnds(gp, col_num, GLP_FR, 0.0, 0.0);
      // z is object function
      glp_set_obj_coef(gp, col_num, 1);

      std::vector<double> val(num_x + 1 + 1, 1);
      std::vector<int>
        ind(boost::make_counting_iterator(std::size_t()),
            boost::make_counting_iterator(num_x + 1 + 1));
      std::size_t row_num = 1;

      // #1 sumation of x_i = p
      // glp_set_row_name(gp, row_num, "p-facilities");
      glp_set_row_bnds(gp, row_num, GLP_FX, p, p);
      glp_set_mat_row(gp, row_num++, num_x, &ind[0], &val[0]);

      for (std::size_t j = 1; j <= num_demand; ++j) {
        // #2 sumation of y_ji for each i = 1
        ind.assign
          (boost::make_counting_iterator(num_x * j),
           boost::make_counting_iterator(num_x * j + num_x + 1 + 1));
        glp_set_row_bnds(gp, row_num, GLP_FX, 1, 1);
        glp_set_mat_row(gp, row_num++, num_x, &ind[0], &val[0]);
      }

      val[1] = -1;
      val[2] = 1;
      for (std::size_t i = 1; i <= num_x; ++i) {
        // #3 x_i >= y_ji for each j => y_ji - x_i <= 0
        ind[1] = i;
        for (std::size_t j = 1; j <= num_demand; ++j) {
          ind[2] = i + num_x * j;
          glp_set_row_bnds(gp, row_num, GLP_UP, 0, 0);
          glp_set_mat_row(gp, row_num++, 2, &ind[0], &val[0]);
        }
      }

      val[num_x + 1] = -1;
      ind[num_x + 1] = num_x + num_y + 1;
      for (std::size_t j = 1; j <= num_demand; ++j) {
        // #4 sumation (y_ji * d_ji) for i - z <= 0
        for (std::size_t i = 1; i <= num_x; ++i) {
          val[i] = d_matrix
            [get(i_map, demands[j - 1])][get(i_map, candidates[i - 1])];
          ind[i] = i + num_x * j;
        }
        glp_set_row_bnds(gp, row_num, GLP_UP, 0, 0);
        glp_set_mat_row(gp, row_num++, num_x + 1, &ind[0], &val[0]);
      }

      return g_prob;
    }

  template <class Graph, class CenterMap, class DistanceMatrix,
            class IndexMap, class CandidateMap, class ExistedCenterMap>
    double glpk_p_center_with_dist_matrix
    (const Graph& g, std::size_t p, CenterMap c_map,
     const DistanceMatrix& d_matrix, IndexMap i_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      using boost::vertices;
      using boost::num_vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      const typename boost::graph_traits<Graph>::vertices_size_type
        num_v = num_vertices(g);

      std::vector<Vertex> candidates;
      candidates.reserve(num_v);
      std::vector<Vertex> demands;
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

      boost::shared_ptr<glp_prob> g_prob
        = make_p_center_prob(p, d_matrix, i_map, candidates, demands);

      // solve p-center problem ===================================
      glp_simplex(g_prob.get(), NULL);
      glp_intopt(g_prob.get(), NULL);

      // set center map
      boost::fill(vertices(g) | Canard::adaptors::mapped(c_map), false);
      for (std::size_t i = 1, num_x = candidates.size(); i <= num_x; ++i) {
        put(c_map, candidates[i - 1], glp_mip_col_val(g_prob.get(), i));
      }

      return glp_mip_obj_val(g_prob.get());
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class ExistedCenterMap, class CandidateMap>
    double glpk_p_center
    (Graph& g, std::size_t p, CenterMap c_map,
     WeightMap w_map, IndexMap i_map,
     ExistedCenterMap existed_center_map, CandidateMap candidate_map)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typename DistMatrix<WeightValue>::type
        d_matrix(create_distance_matrix(g, w_map, i_map));
      set_distance_matrix<WeightValue>(g, d_matrix, i_map, existed_center_map);

      return glpk_p_center_with_dist_matrix(g, p, c_map,
          d_matrix, i_map, existed_center_map, candidate_map);
    }

  template <class Graph, class CenterMap, class WeightMap,
            class IndexMap, class ExistedCenterMap>
    inline double glpk_p_center
    (const Graph& g, std::size_t p, CenterMap c_map,
     WeightMap w_map, IndexMap i_map, ExistedCenterMap existed_center)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return glpk_p_center(
          g, p, c_map, w_map, i_map, existed_center,
          boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    inline double glpk_p_center
    (const Graph& g, std::size_t p, CenterMap c_map,
     WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return glpk_p_center(
          g, p, c_map, w_map, i_map,
          boost::make_constant_property<Vertex>(false));
    }

  template <class Graph, class CenterMap, class WeightMap>
    inline double glpk_p_center
    (const Graph& g, std::size_t p, CenterMap c_map, WeightMap w_map)
    {
      return glpk_p_center(
          g, p, c_map, w_map, get(boost::vertex_index, g));
    }

  template <class Graph, class CenterMap, class WeightMap>
    inline double glpk_p_center
    (const Graph& g, std::size_t p, CenterMap c_map)
    {
      return glpk_p_center(
          g, p, c_map, get(boost::edge_weight, g));
    }
}

#endif  // CANARD_GLPK_P_CENTER_H

