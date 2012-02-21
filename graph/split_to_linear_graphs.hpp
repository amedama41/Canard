#ifndef CANARD_SPLIT_TO_LINEAR_GRAPHS_H
#define CANARD_SPLIT_TO_LINEAR_GRAPHS_H

#include <set>
#include <map>
#include <boost/graph/undirected_dfs.hpp>
#include <boost/graph/properties.hpp>
#include "Canard/graph/linear_graph_container.hpp"

namespace Canard {
  // TODO GraphConcept(VList,Incident,..)
  namespace detail {
    template <class Graph, class VertexSet, class EdgeSet>
      inline typename LinearGraph<Graph>::type
      make_linear_graph
      (const Graph& g, EdgeSet& e_set, VertexSet& v_set)
      {
        return boost::make_filtered_graph
          (g, is_in_subset<EdgeSet>(e_set), is_in_subset<VertexSet>(v_set));
      }

    template <class Graph, class SplitVertexMap, class EventType>
      class edge_event
      : public boost::base_visitor<
          edge_event<Graph, SplitVertexMap, EventType>
      > {
          typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
          typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
        public:
          typedef EventType event_filter;

          edge_event(LinearGraphs<Graph> &lgs, SplitVertexMap sv_map,
              std::set<Edge>& e_set, std::set<Vertex>& v_set)
            : linear_graphs(&lgs),
              split_vertex_map(sv_map),
              e_set(&e_set), v_set(&v_set) { }

          void operator()(const Edge e, const Graph& g)
          {
            using boost::out_degree;
            using boost::source;
            using boost::target;
            const Vertex u = source(e, g);
            const Vertex v = target(e, g);
            v_set->insert(u);
            e_set->insert(e);
            if (out_degree(v, g) != 2 || get(split_vertex_map, v) != false) {
              v_set->insert(v);
              linear_graphs->push_back(make_linear_graph(g, *e_set, *v_set));
            }
          }

        private:
          LinearGraphs<Graph>* linear_graphs;
          SplitVertexMap split_vertex_map;
          std::set<Edge>*   e_set;
          std::set<Vertex>* v_set;
      };
  }

  template <class Graph, class EdgeColorMap, class SplitVertexMap>
    LinearGraphs<Graph>
    split_to_linear_graphs
    (const Graph& g, EdgeColorMap color_map, SplitVertexMap split_vertex_map)
    {
      using boost::vertices;
      using boost::out_degree;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::edge_descriptor   Edge;

      // find start vertex
      Vertex s;
      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        if (out_degree(*vi, g) != 2 || get(split_vertex_map, *vi) != false) {
          s = *vi;
          break;
        }
      }

      LinearGraphs<Graph> linear_graphs;
      std::set<Edge>   e_set;
      std::set<Vertex> v_set;

      boost::undirected_dfs(g,
          boost::visitor(boost::make_dfs_visitor
            (std::make_pair
             (detail::edge_event<Graph, SplitVertexMap, boost::on_tree_edge>
              (linear_graphs, split_vertex_map, e_set, v_set),
              detail::edge_event<Graph, SplitVertexMap, boost::on_back_edge>
              (linear_graphs, split_vertex_map, e_set, v_set)))).
          root_vertex(s). edge_color_map(color_map));

      return linear_graphs;
    }

  template <class Graph, class EdgeColorMap>
    inline LinearGraphs<Graph>
    split_to_linear_graphs
    (const Graph& g, EdgeColorMap color_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return split_to_linear_graphs
        (g, color_map, boost::make_constant_property<Vertex>(false));
    }

  template <class Graph>
    inline LinearGraphs<Graph>
    split_to_linear_graphs
    (const Graph& g)
    {
      typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
      std::map<Edge, boost::default_color_type> color;
      return split_to_linear_graphs(g, boost::make_assoc_property_map(color));
    }
}

#endif  // CANARD_SPLIT_TO_LINEAR_GRAPHS_H

