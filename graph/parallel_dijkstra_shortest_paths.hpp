#ifndef CANARD_PARALLEL_DIJKSTRA_SHORTEST_PATHS_H
#define CANARD_PARALLEL_DIJKSTRA_SHORTEST_PATHS_H

#include <vector>
#include <functional>
#include <iterator>
#include <boost/concept_check.hpp>
#include <boost/limits.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/relax.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/pending/relaxed_heap.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/graph/two_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/type_traits.hpp>
#include <boost/graph/visitors.hpp>

namespace Canard {
  // Generator recorder for Network Voronoi diagram
  template <class GeneratorMap, class Tag>
  struct GeneratorRecorder
    : public boost::base_visitor<GeneratorRecorder<GeneratorMap, Tag> >
  {
    typedef Tag event_filter;
    GeneratorRecorder(GeneratorMap pa) : m_generator(pa) { }
    template <class Edge, class Graph>
    void operator()(Edge e, const Graph& g) {
      using boost::source;
      using boost::target;
      put(m_generator, target(e, g), get(m_generator, source(e, g)));
    }
    GeneratorMap m_generator;
  };
  template <class GeneratorMap, class Tag>
  GeneratorRecorder<GeneratorMap, Tag>
  record_generators(GeneratorMap pa, Tag) {
    return GeneratorRecorder<GeneratorMap, Tag> (pa);
  }

  // Call breadth first search
  template <class Graph, class RootVertexMap, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero, class ColorMap>
  inline void
  parallel_dijkstra_shortest_paths_no_init
    (const Graph& g, RootVertexMap root_vertex_map,
     PredecessorMap predecessor_map, DistanceMap distance_map,
     WeightMap weight_map, IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis, ColorMap color_map)
  {
    using boost::vertices;
    using boost::num_vertices;
    // TODO VertexList and IncidentGraph CenceptCheck
    typedef
      typename boost::property_traits<RootVertexMap>::value_type
      RootVertex;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    boost::function_requires< boost::Convertible<RootVertex, Vertex> >();

    typedef boost::indirect_cmp<DistanceMap, Compare> IndirectCmp;
    IndirectCmp icmp(distance_map, compare);

#ifdef BOOST_GRAPH_DIJKSTRA_USE_RELAXED_HEAP
    typedef boost::relaxed_heap<Vertex, IndirectCmp, IndexMap> MutableQueue;
    MutableQueue Q(num_vertices(g), icmp, index_map);
#else // Now the default: use a d-ary heap
    boost::scoped_array<std::size_t> index_in_heap_map_holder;
    typedef
      boost::detail::vertex_property_map_generator<Graph, IndexMap, std::size_t>
      IndexInHeapMapHelper;
    typedef typename IndexInHeapMapHelper::type IndexInHeapMap;
    IndexInHeapMap index_in_heap =
      IndexInHeapMapHelper::build(g, index_map, index_in_heap_map_holder);
    typedef
      boost::d_ary_heap_indirect<Vertex, 4, IndexInHeapMap, DistanceMap, Compare>
      MutableQueue;
    MutableQueue Q(distance_map, index_in_heap, compare);
#endif // Relaxed heap

    typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
      if (get(root_vertex_map, *vi) != false) {
        Q.push(*vi);
      }
    }
    if (Q.empty()) {
      // TODO
      throw std::runtime_error("not specified root vertices.");
    }

    boost::detail::dijkstra_bfs_visitor<DijkstraVisitor, MutableQueue,
      WeightMap, PredecessorMap, DistanceMap, Combine, Compare>
        bfs_vis(vis, Q, weight_map, predecessor_map, distance_map,
            combine, compare, zero);

    const Vertex s = Q.top();
    Q.pop();
    boost::breadth_first_visit(g, s, Q, bfs_vis, color_map);
  }

  // Call breadth first search with default color map.
  template <class Graph, class RootVertexMap, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero>
  inline void
  parallel_dijkstra_shortest_paths_no_init
    (const Graph& g, RootVertexMap root_vertex_map,
     PredecessorMap predecessor_map, DistanceMap distance_map,
     WeightMap weight_map, IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis)
  {
    typedef
      boost::detail::default_color_map_generator<Graph, IndexMap>
      ColorMapHelper;
    typedef typename ColorMapHelper::type ColorMap;
    ColorMap color_map = ColorMapHelper::build(g, index_map);

    parallel_dijkstra_shortest_paths_no_init
      (g, root_vertex_map, predecessor_map, distance_map, weight_map,
       index_map, compare, combine, zero, vis, color_map);
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class RootVertexMap, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, class ColorMap>
  inline void
  parallel_dijkstra_shortest_paths
    (const VertexListGraph& g,
     RootVertexMap root_vertex_map,
     PredecessorMap predecessor_map, DistanceMap distance_map,
     WeightMap weight_map, IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis, ColorMap color_map)
  {
    using boost::vertices;
    // TODO VertexList and IncidentGraph CenceptCheck
    typedef
      typename boost::property_traits<RootVertexMap>::value_type
      RootVertex;
    typedef
      typename boost::graph_traits<VertexListGraph>::vertex_descriptor
      Vertex;
    boost::function_requires< boost::Convertible<RootVertex, Vertex> >();

    typedef typename boost::property_traits<ColorMap>::value_type ColorValue;
    typedef boost::color_traits<ColorValue> Color;
    typename boost::graph_traits<VertexListGraph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
      vis.initialize_vertex(*vi, g);
      if (get(root_vertex_map, *vi) != false) {
        put(distance_map, *vi, zero);
      }
      else {
        put(distance_map, *vi, inf);
      }
      put(predecessor_map, *vi, *vi);
      put(color_map, *vi, Color::white());
    }

    parallel_dijkstra_shortest_paths_no_init
      (g, root_vertex_map, predecessor_map, distance_map,
       weight_map, index_map, compare, combine, zero, vis, color_map);
  }

  using namespace boost;
  // Initialize distances and call breadth first search with default color map
  template <class VertexListGraph, class RootVertexMap, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, typename T, typename Tag, 
            typename Base>
  inline void
  parallel_dijkstra_shortest_paths
    (const VertexListGraph& g,
     RootVertexMap root_vertex_map,
     PredecessorMap predecessor_map, DistanceMap distance_map,
     WeightMap weight_map, IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis,
     const boost::bgl_named_params<T, Tag, Base>&
     BOOST_GRAPH_ENABLE_IF_MODELS_PARM(VertexListGraph,boost::vertex_list_graph_tag))
  {
    using boost::num_vertices;
    boost::two_bit_color_map<IndexMap> color_map(num_vertices(g), index_map);
    parallel_dijkstra_shortest_paths
      (g, root_vertex_map, predecessor_map, distance_map, weight_map, index_map,
       compare, combine, inf, zero, vis,
       color_map);
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class RootVertexMap, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero>
  inline void
  parallel_dijkstra_shortest_paths
    (const VertexListGraph& g,
     RootVertexMap root_vertex_map,
     PredecessorMap predecessor_map, DistanceMap distance_map,
     WeightMap weight_map, IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis)
  {
    parallel_dijkstra_shortest_paths
      (g, root_vertex_map, predecessor_map, distance_map, weight_map, index_map,
       compare, combine, inf, zero, vis,
       boost::no_named_parameters());
  }

  namespace detail {

    // Handle defaults for PredecessorMap and
    // Distance Compare, Combine, Inf and Zero
    template <class VertexListGraph, class RootVertexMap, class DistanceMap,
              class WeightMap, class IndexMap, class Params>
    inline void
    parallel_dijkstra_dispatch2
      (const VertexListGraph& g,
       RootVertexMap root_vertex_map,
       DistanceMap distance_map, WeightMap weight_map, IndexMap index_map,
       const Params& params)
    {
      // Default for predecessor map
      boost::dummy_property_map p_map;

      typedef typename boost::property_traits<DistanceMap>::value_type D;
      parallel_dijkstra_shortest_paths
        (g, root_vertex_map,
         choose_param(get_param(params, boost::vertex_predecessor), p_map),
         distance_map, weight_map, index_map,
         choose_param(get_param(params, boost::distance_compare_t()),
                      std::less<D>()),
         choose_param(get_param(params, boost::distance_combine_t()),
                      boost::closed_plus<D>()),
         choose_param(get_param(params, boost::distance_inf_t()),
                      (std::numeric_limits<D>::max)()),
         choose_param(get_param(params, boost::distance_zero_t()),
                      D()),
         choose_param(get_param(params, boost::graph_visitor),
                      boost::make_dijkstra_visitor(boost::null_visitor())),
         params);
    }

    template <class VertexListGraph, class RootVertexMap, class DistanceMap,
              class WeightMap, class IndexMap, class Params>
    inline void
    parallel_dijkstra_dispatch1
      (const VertexListGraph& g,
       RootVertexMap root_vertex_map,
       DistanceMap distance_map, WeightMap weight_map, IndexMap index_map,
       const Params& params)
    {
      using boost::num_vertices;
      // Default for distance map
      typedef typename boost::property_traits<WeightMap>::value_type D;
      const typename std::vector<D>::size_type
        n = boost::is_default_param(distance_map) ? num_vertices(g) : 1;
      std::vector<D> d_map(n);

      detail::parallel_dijkstra_dispatch2
        (g, root_vertex_map, choose_param
         (distance_map,
          boost::make_iterator_property_map
          (d_map.begin(), index_map, d_map[0])),
         weight_map, index_map, params);
    }
  } // namespace detail

  // Named Parameter Variant
  template <class VertexListGraph, class RootVertexMap,
           class Param, class Tag, class Rest>
  inline void
  parallel_dijkstra_shortest_paths
    (const VertexListGraph& g,
     RootVertexMap root_vertex_map,
     const boost::bgl_named_params<Param,Tag,Rest>& params)
  {
    // Default for edge weight and vertex index map is to ask for them
    // from the graph.  Default for the visitor is null_visitor.
    detail::parallel_dijkstra_dispatch1
      (g, root_vertex_map,
       get_param(params, boost::vertex_distance),
       choose_const_pmap
       (get_param(params, boost::edge_weight), g, boost::edge_weight),
       choose_const_pmap
       (get_param(params, boost::vertex_index), g, boost::vertex_index),
       params);
  }

} // namespace Canard

#endif // CANARD_PARALLEL_DIJKSTRA_SHORTEST_PATHS_H

