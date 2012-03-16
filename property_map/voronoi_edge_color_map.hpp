#ifndef CANARD_VORONOI_EDGE_COLOR_MAP_HPP
#define CANARD_VORONOI_EDGE_COLOR_MAP_HPP

#include <boost/property_map/property_map.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace Canard {
  template <class Graph, class VertexColorMap>
  class voronoi_edge_color_map
    : public boost::put_get_helper<
        typename boost::remove_reference<
          typename boost::property_traits<VertexColorMap>::reference
        >::type const&,
        voronoi_edge_color_map<Graph, VertexColorMap> >
  {
  public:
    typedef typename boost::graph_traits<Graph>::edge_descriptor key_type;
    typedef typename boost::property_traits<VertexColorMap>::value_type
      value_type;
    typedef typename boost::remove_reference<
      typename boost::property_traits<VertexColorMap>::reference
    >::type const& reference;
    typedef boost::readable_property_map_tag category;

  public:
    voronoi_edge_color_map
    (Graph const& graph, VertexColorMap const& vertex_color_map)
      : graph_(&graph),
        vertex_color_map_(vertex_color_map),
        default_color_(value_type())
    {
    }

    voronoi_edge_color_map
    (Graph const& graph,
     VertexColorMap const& vertex_color_map, value_type const& def_color)
      : graph_(&graph),
        vertex_color_map_(vertex_color_map),
        default_color_(def_color)
    {
    }

    reference operator[](key_type const& e) const
    {
      using boost::source;
      using boost::target;
      reference color1 = get(vertex_color_map_, source(e, *graph_));
      reference color2 = get(vertex_color_map_, target(e, *graph_));
      if (color1 == color2) {
        return color1;
      }
      return default_color_;
    }

    reference operator()(key_type const& e) const
    {
      return (*this)[e];
    }

  private:
    Graph const *graph_;
    VertexColorMap vertex_color_map_;
    value_type default_color_;
  };

  template <class Graph, class VertexColorMap>
  voronoi_edge_color_map<Graph, VertexColorMap>
  make_voronoi_edge_color_map
  (Graph const& graph, VertexColorMap const& vertex_color_map,
   typename boost::property_traits<VertexColorMap>::value_type const& def_color
     = typename boost::property_traits<VertexColorMap>::value_type())
  {
    typedef voronoi_edge_color_map<Graph, VertexColorMap> PMap;
    return PMap(graph, vertex_color_map, def_color);
  }
}

#endif  // CANARD_VORONOI_EDGE_COLOR_MAP_HPP

