#ifndef CANARD_TREE_EDGE_COLOR_MAP_HPP
#define CANARD_TREE_EDGE_COLOR_MAP_HPP

#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace Canard {
  template <class Graph, class PredecessorMap, class Color>
  class tree_edge_color_map
    : public boost::put_get_helper<
        Color const&,
        tree_edge_color_map<Graph, PredecessorMap, Color> >
  {
  public:
    typedef typename boost::graph_traits<Graph>::edge_descriptor key_type;
    typedef Color value_type;
    typedef Color const& reference;
    typedef boost::readable_property_map_tag category;

  public:
    tree_edge_color_map
    (Graph const& graph, PredecessorMap const& predecessor_map,
     Color const& tree_edge_color)
      : graph_(&graph),
        predecessor_map_(predecessor_map),
        tree_edge_color_(tree_edge_color),
        co_tree_edge_color_(Color())
    {
    }

    tree_edge_color_map
    (Graph const& graph, PredecessorMap const& predecessor_map,
     Color const& tree_edge_color, Color const& co_tree_edge_color)
      : graph_(&graph),
        predecessor_map_(predecessor_map),
        tree_edge_color_(tree_edge_color),
        co_tree_edge_color_(co_tree_edge_color)
    {
    }

    reference operator[](key_type const& e) const
    {
      using boost::source;
      using boost::target;

      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      Vertex const u = source(e, *graph_);
      Vertex const v = target(e, *graph_);

      if (get(predecessor_map_, u) == v || get(predecessor_map_, v) == u) {
        return tree_edge_color_;
      }
      return co_tree_edge_color_;
    }

    reference operator()(key_type const& e) const
    {
      return (*this)[e];
    }

  private:
    Graph const *graph_;
    PredecessorMap predecessor_map_;
    Color tree_edge_color_;
    Color co_tree_edge_color_;
  };

  template <class Graph, class PredecessorMap, class Color>
  tree_edge_color_map<Graph, PredecessorMap, Color>
  make_tree_edge_color_map
  (Graph const& graph, PredecessorMap const& predecessor_map,
   Color const& tree_edge_color, Color const& co_tree_edge_color)
  {
    return tree_edge_color_map<Graph, PredecessorMap, Color>(
        graph, predecessor_map, tree_edge_color, co_tree_edge_color);
  }

  template <class Graph, class PredecessorMap, class Color>
  tree_edge_color_map<Graph, PredecessorMap, Color>
  make_tree_edge_color_map
  (Graph const& graph, PredecessorMap const& predecessor_map,
   Color const& tree_edge_color)
  {
    return tree_edge_color_map<Graph, PredecessorMap, Color>(
        graph, predecessor_map, tree_edge_color, Color());
  }
} // namespace Canard

#endif  // CANARD_TREE_EDGE_COLOR_MAP_HPP

