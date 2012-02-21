#ifndef CANARD_CALC_UNEVEN_DISTRIBUTION_HPP
#define CANARD_CALC_UNEVEN_DISTRIBUTION_HPP

#include <vector>
#include <numeric>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/range/algorithm.hpp>
#include "Canard/graph/parallel_dijkstra_shortest_paths.hpp"
#include "Canard/property_map/property_map_copy.hpp"

namespace Canard {
  template <class Graph, class CenterMap,
            class GeneratorMap, class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class ColorMap>
    double calc_uneven_distribution
    (const Graph& g, CenterMap center_map,
     GeneratorMap g_map, PredecessorMap p_map, DistanceMap d_map,
     WeightMap w_map, IndexMap i_map, ColorMap c_map)
    {
      using boost::vertices;
      using boost::edges;
      using boost::source;
      using boost::target;

      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      boost::for_each(vertices(g),
          make_property_map_copy(
            g_map, boost::typed_identity_property_map<Vertex>()));

      parallel_dijkstra_shortest_paths(g, center_map,
          boost::predecessor_map(p_map). distance_map(d_map).
          weight_map(w_map). vertex_index_map(i_map). color_map(c_map).
          visitor(boost::make_dijkstra_visitor(
              record_generators(g_map, boost::on_edge_relaxed()))));

      typedef typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      std::vector<WeightValue> dist;
      typename boost::graph_traits<Graph>::edge_iterator ei, e_end;
      for (boost::tie(ei, e_end) = edges(g); ei != e_end; ++ei) {
        typename boost::graph_traits<Graph>::vertex_descriptor
          u = source(*ei, g), v = target(*ei, g);
#ifdef NEW_UNEVEN_DISTRIBUTION
        if (get(p_map, u) != v && get(p_map, v) != u)
#else
        if (get(g_map, u) != get(g_map, v))
#endif
        {
          dist.push_back(get(d_map, u) + get(d_map, v) + get(w_map, *ei));
        }
      }

      const WeightValue average = std::accumulate(
          dist.begin(), dist.end(),
          WeightValue()) / dist.size();
      std::transform(
          dist.begin(), dist.end(),
          dist.begin(),
          boost::lambda::_1 - average);
      return std::inner_product(
          dist.begin(), dist.end(),
          dist.begin(), WeightValue()) / 2;
    }

  template <class Graph, class CenterMap,
            class GeneratorMap,
            class WeightMap, class IndexMap, class ColorMap,
            class Param, class Tag, class Rest>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_uneven_distribution
    (const Graph& g, CenterMap center_map,
     GeneratorMap g_map,
     WeightMap weight_map, IndexMap index_map, ColorMap color_map,
     const boost::bgl_named_params<Param, Tag, Rest>& params)
    {
      using boost::num_vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      const typename boost::graph_traits<Graph>::vertices_size_type
        n = num_vertices(g);
      std::vector<Vertex> predecessor(n);
      std::vector<WeightValue> distance(n);
      return calc_uneven_distribution(
          g, center_map, g_map,
          boost::choose_param(
            boost::get_param(params, boost::vertex_predecessor),
            make_iterator_property_map(
              predecessor.begin(), index_map, predecessor[0])),
          boost::choose_param(boost::get_param(params, boost::vertex_distance),
            make_iterator_property_map(
              distance.begin(), index_map, distance[0])),
          weight_map, index_map, color_map);
    }

  template <class Graph, class CenterMap,
            class WeightMap, class IndexMap, class ColorMap,
            class Param, class Tag, class Rest>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_uneven_distribution
    (const Graph& g, CenterMap center_map,
     WeightMap weight_map, IndexMap index_map, ColorMap color_map,
     const boost::bgl_named_params<Param, Tag, Rest>& params)
    {
      using boost::num_vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      std::vector<Vertex> generator(num_vertices(g));
      return calc_uneven_distribution(g, center_map,
          boost::make_iterator_property_map(
            generator.begin(), index_map, generator[0]),
          weight_map, index_map, color_map,
          params);
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class Param, class Tag, class Rest>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_uneven_distribution
    (const Graph& g, CenterMap center_map,
     WeightMap weight_map, IndexMap index_map,
     const boost::bgl_named_params<Param, Tag, Rest>& params)
    {
      using boost::num_vertices;
      boost::two_bit_color_map<IndexMap> color_map(num_vertices(g), index_map);
      return calc_uneven_distribution(
          g, center_map, weight_map, index_map, color_map, params);
    }

  template <class Graph, class CenterMap, class WeightMap,
            class Param, class Tag, class Rest>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_uneven_distribution
    (const Graph& g, CenterMap center_map, WeightMap weight_map,
     const boost::bgl_named_params<Param, Tag, Rest>& params)
    {
      return calc_uneven_distribution(
          g, center_map, weight_map, get(boost::vertex_index, g), params);
    }

  template <class Graph, class CenterMap, class WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    calc_uneven_distribution
    (const Graph& g, CenterMap center_map, WeightMap weight_map)
    {
      return calc_uneven_distribution(
          g, center_map, weight_map,
          boost::visitor(boost::make_dijkstra_visitor(boost::null_visitor())));
    }
}

#endif  // CANARD_CALC_UNEVEN_DISTRIBUTION_HPP

