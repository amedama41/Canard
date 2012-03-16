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
        return boost::make_filtered_graph(
            g, is_in_subset<EdgeSet>(e_set), is_in_subset<VertexSet>(v_set));
      }

    template <class Graph, class SplitVertexMap, class EventType>
      class edge_event
        : public boost::base_visitor<
            edge_event<Graph, SplitVertexMap, EventType>
          >
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef EventType event_filter;

        edge_event(linear_graphs<Graph> &lgs, SplitVertexMap sv_map,
            std::set<Edge>& e_set, std::set<Vertex>& v_set)
          : lgraphs_(&lgs),
            split_vertex_map_(sv_map),
            e_set_(&e_set), v_set_(&v_set) { }

        void operator()(const Edge e, const Graph& g)
        {
          using boost::out_degree;
          using boost::source;
          using boost::target;
          const Vertex u = source(e, g);
          const Vertex v = target(e, g);
          v_set_->insert(u);
          e_set_->insert(e);
          if (out_degree(v, g) != 2 || get(split_vertex_map_, v) != false) {
            v_set_->insert(v);
            lgraphs_->push_back(make_linear_graph(g, *e_set_, *v_set_));
          }
        }

      private:
        linear_graphs<Graph>* lgraphs_;
        SplitVertexMap split_vertex_map_;
        std::set<Edge>*   e_set_;
        std::set<Vertex>* v_set_;
      };
  }

  template <class Graph, class EdgeColorMap, class SplitVertexMap>
    linear_graphs<Graph>
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

      linear_graphs<Graph> lgraphs;
      std::set<Edge>   e_set;
      std::set<Vertex> v_set;

      boost::undirected_dfs(g,
          boost::visitor(boost::make_dfs_visitor(
              std::make_pair(
                detail::edge_event<Graph, SplitVertexMap, boost::on_tree_edge>(
                  lgraphs, split_vertex_map, e_set, v_set),
                detail::edge_event<Graph, SplitVertexMap, boost::on_back_edge>(
                  lgraphs, split_vertex_map, e_set, v_set)))).
          root_vertex(s). edge_color_map(color_map));

      return lgraphs;
    }

  template <class Graph, class EdgeColorMap>
    inline linear_graphs<Graph>
    split_to_linear_graphs
    (const Graph& g, EdgeColorMap color_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return split_to_linear_graphs(
          g, color_map, boost::make_constant_property<Vertex>(false));
    }

  template <class Graph>
    inline linear_graphs<Graph>
    split_to_linear_graphs
    (const Graph& g)
    {
      typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
      std::map<Edge, boost::default_color_type> color;
      return split_to_linear_graphs(g, boost::make_assoc_property_map(color));
    }
}

#endif  // CANARD_SPLIT_TO_LINEAR_GRAPHS_H

