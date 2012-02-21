#ifndef CANARD_GRAPH_UTILITY_H
#define CANARD_GRAPH_UTILITY_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/copy.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/tuple/tuple.hpp>

namespace Canard {
  template <class InputGraph, class OutputGraph, class WeightTag,
            class IndexMap, class Orig2CopyMap>
    void add_middle_vertex_to_edges
    (const InputGraph& in_g, OutputGraph& out_g, WeightTag w_tag,
     IndexMap i_map, Orig2CopyMap orig_to_copy_map)
    {
      using boost::vertices;
      using boost::edges;
      using boost::num_edges;
      using boost::source;
      using boost::target;
      using boost::clear_vertex;
      using boost::add_vertex;
      using boost::add_edge;

      typedef typename boost::graph_traits<OutputGraph>::vertex_descriptor
        Vertex;
      typedef typename boost::property_map<OutputGraph, boost::edge_all_t>::type
        EdgePropertyMap;
      typedef typename boost::property_traits<EdgePropertyMap>::value_type
        EdgePropertyValue;
      typedef typename boost::property_map<OutputGraph, WeightTag>::type
        WeightMap;
      typedef typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      typedef boost::tuple<Vertex, Vertex, EdgePropertyValue, WeightValue>
        EdgeTuple;
      typedef std::vector<EdgeTuple> EdgeTuples;

      boost::copy_graph(in_g, out_g,
          boost::vertex_index_map(i_map). orig_to_copy(orig_to_copy_map));

      WeightMap w_map = get(w_tag, out_g);
      EdgePropertyMap e_all_map = get(boost::edge_all, out_g);
      EdgeTuples edge_tuples;
      edge_tuples.reserve(num_edges(out_g));

      typename boost::graph_traits<OutputGraph>::edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = edges(out_g); ei != ei_end; ++ei) {
        edge_tuples.push_back
          (make_tuple(source(*ei, out_g), target(*ei, out_g),
                      get(e_all_map, *ei), get(w_map, *ei) / 2));
      }

      typename boost::graph_traits<OutputGraph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi, vi_end) = vertices(out_g); vi != vi_end; ++vi) {
        clear_vertex(*vi, out_g);
      }

      // add_vertex must not invalid vertex_descriptor
      for (typename EdgeTuples::const_iterator eti = edge_tuples.begin(),
          eti_end = edge_tuples.end(); eti != eti_end; ++eti) {
        const Vertex new_v = add_vertex(out_g);
        put(w_map,
            add_edge(get<0>(*eti), new_v, get<2>(*eti), out_g).first,
            get<3>(*eti));
        put(w_map,
            add_edge(new_v, get<1>(*eti), get<2>(*eti), out_g).first,
            get<3>(*eti));
      }
    }
}

#endif  // CANARD_GRAPH_UTILITY_H

