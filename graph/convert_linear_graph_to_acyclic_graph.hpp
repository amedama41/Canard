#ifndef CANARD_CONVERT_LINEAR_GRAPH_TO_ACYCLIC_GRAPH_H
#define CANARD_CONVERT_LINEAR_GRAPH_TO_ACYCLIC_GRAPH_H

#include <vector>
#include <iterator>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/relax.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/distance.hpp>

namespace Canard {
  template <class Graph, class WeightMap>
    struct AcyclicGraph {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typedef boost::compressed_sparse_row_graph<
          boost::directedS, Vertex, WeightValue
        > type;
    };

  namespace detail {
    template <class Graph>
      inline typename boost::graph_traits<Graph>::vertex_descriptor
      get_start_vertex(const Graph& g)
      {
        using boost::vertices;
        using boost::out_degree;
        typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
          if (out_degree(*vi, g) == 1) {
            return *vi;
          }
        }
        // TODO error process
        throw std::runtime_error("Graph is not linear.");
      }

    template <class WeightValue, class Combine, class Multiple>
      inline std::vector<WeightValue>
      convert_edge_weight
      (std::vector<WeightValue>& weight, Combine combine, Multiple multiple)
      {
        typedef std::vector<WeightValue> WeightVector;

        const std::size_t num_e = weight.size();
        WeightVector new_weight;
        new_weight.reserve((num_e + 1) * num_e / 2);

        typename WeightVector::const_iterator first = weight.begin();
        typename WeightVector::const_iterator last  = weight.end();
        std::back_insert_iterator<WeightVector>
          dest = std::back_inserter(new_weight);
        for (; first != last; ++first) {
          dest = std::partial_sum(first, last, dest, combine);
        }

        boost::transform(new_weight, new_weight.begin(),
           boost::lambda::bind(multiple, boost::lambda::_1, boost::lambda::_1));

        return new_weight;
      }

    inline std::vector< std::pair<std::size_t, std::size_t> >
      create_new_edges(std::size_t num_v)
      {
        typedef std::pair<std::size_t, std::size_t> Edge;
        std::vector<Edge> new_edges;
        new_edges.reserve(num_v * (num_v - 1) / 2);
        std::back_insert_iterator< std::vector<Edge> >
          dest = std::back_inserter(new_edges);
        for (std::size_t i = 0; i < num_v; ++i) {
          dest = boost::transform(
              std::make_pair(
                boost::counting_iterator<std::size_t>(i + 1),
                boost::counting_iterator<std::size_t>(num_v)),
              // boost::counting_range<std::size_t>(i + 1, num_v),
              dest,
              boost::lambda::bind(
                boost::lambda::constructor<Edge>(), i, boost::lambda::_1));
        }
        return new_edges;
      }
  }

  template <class LinearGraph, class WeightMap, class CandidateMap,
            class Combine, class Multiple>
    typename AcyclicGraph<LinearGraph, WeightMap>::type
    convert_linear_graph_to_acyclic_graph
    (const LinearGraph& g, WeightMap weight, CandidateMap candidate,
     Combine combine, Multiple multiple)
    {
      using boost::vertices;
      using boost::out_edges;
      using boost::out_degree;
      using boost::target;
      typedef boost::graph_traits<LinearGraph> Traits;
      typedef typename Traits::vertex_descriptor Vertex;
      typedef typename Traits::vertex_iterator VIter;
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typedef typename AcyclicGraph<LinearGraph, WeightMap>::type AcyclicGraph;

      const typename std::iterator_traits<VIter>::difference_type
        num_v = boost::distance(vertices(g));
      const Vertex s = detail::get_start_vertex(g);

      std::vector<Vertex> new_vertices;
      new_vertices.reserve(num_v);
      std::vector<WeightValue> e_weight;
      e_weight.reserve(num_v - 1);

      // Collect info about a linear graph
      new_vertices.push_back(s);
      Vertex prev = s, now = s;
      typename Traits::out_edge_iterator ei, ei_end;
      do {
        boost::tie(ei, ei_end) = out_edges(now, g);
        if (target(*ei, g) == prev) {
          ++ei;
        }
        const Vertex next = target(*ei, g);
        if (get(candidate, next) || out_degree(next, g) == 1) {
          new_vertices.push_back(next);
        }

        if (get(candidate, now) || out_degree(now, g) == 1) {
          e_weight.push_back(get(weight, *ei));
        }
        else {
          e_weight.back() = combine(e_weight.back(), get(weight, *ei));
        }
        prev = now;
        now = next;
      } while (out_degree(now, g) != 1);

      // create acyclic graph's edge info
      const std::vector< std::pair<std::size_t, std::size_t> >
        new_edges(detail::create_new_edges(new_vertices.size()));
      const std::vector<WeightValue>
        new_weight(detail::convert_edge_weight(e_weight, combine, multiple));

      AcyclicGraph acyclic_g(
          boost::edges_are_sorted,
          new_edges.begin(), new_edges.end(),
          new_weight.begin(), new_vertices.size());

      // set vertex prop;
      typename boost::graph_traits<AcyclicGraph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi, vi_end) = vertices(acyclic_g); vi != vi_end; ++vi) {
        acyclic_g[*vi] = new_vertices[get(boost::vertex_index, acyclic_g, *vi)];
      }

      return acyclic_g;
    }

  template <class LinearGraph, class WeightMap, class CandidateMap>
    inline typename AcyclicGraph<LinearGraph, WeightMap>::type
    convert_linear_graph_to_acyclic_graph
    (const LinearGraph& g, WeightMap weight, CandidateMap candidate)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return convert_linear_graph_to_acyclic_graph(
          g, weight, candidate,
          boost::closed_plus<WeightValue>(), std::multiplies<WeightValue>());
    }

  template <class LinearGraph, class WeightMap>
    inline typename AcyclicGraph<LinearGraph, WeightMap>::type
    convert_linear_graph_to_acyclic_graph
    (const LinearGraph& g, WeightMap weight)
    {
      typedef typename boost::graph_traits<LinearGraph>::vertex_descriptor
        Vertex;
      return convert_linear_graph_to_acyclic_graph(
          g, weight, boost::make_constant_property<Vertex>(true));
    }
}

#endif	// CANARD_CONVERT_LINEAR_GRAPH_TO_ACYCLIC_GRAPH_H

