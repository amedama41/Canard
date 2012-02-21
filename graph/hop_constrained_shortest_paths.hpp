#ifndef CANARD_HOP_CONSTRAINED_SHORTEST_PATHS_HPP
#define CANARD_HOP_CONSTRAINED_SHORTEST_PATHS_HPP

#include <vector>
#include <functional>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>

namespace Canard {
  template <class Graph, class WeightMap,
            class Compare = std::less<
              typename boost::property_traits<WeightMap>::value_type> >
    struct HopConstrainedClass {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;

      struct ResourceCont
      {
        WeightValue weight;
        std::size_t hop;
        Compare compare;

        ResourceCont
        (const WeightValue w = WeightValue(),
         std::size_t h = 0, const Compare c = Compare())
          : weight(w), hop(h), compare(c) { }
        bool operator<(const ResourceCont& rhs) const
        {
          return (hop == rhs.hop) ? compare(weight, rhs.weight) : false;
        }
        bool operator==(const ResourceCont& rhs) const
        {
          return (hop == rhs.hop) &&
            (!compare(weight, rhs.weight) && !compare(rhs.weight, weight));
        }
      };

      template <class Combine>
      class ResourceExtensionFunc
      {
      public:
        ResourceExtensionFunc
        (Vertex t, WeightMap w, std::size_t h, Combine c)
          : target_v(t), w_map(w), max_hop(h), combine(c) { }
        inline bool operator()
        (const Graph& g, ResourceCont& new_cont,
         const ResourceCont& old_cont, Edge ed) const
        {
          new_cont.weight = combine(old_cont.weight, get(w_map, ed));
          new_cont.hop = old_cont.hop + 1;

          if (target(ed, g) == target_v) {
            return new_cont.hop == max_hop;
          }
          return (new_cont.hop < max_hop);
        }
      private:
        Vertex target_v;
        WeightMap w_map;
        std::size_t max_hop;
        Combine combine;
      };

      class DominanceFunc
      {
      public:
        DominanceFunc(std::size_t h = 0)
          : max_hop(h) { }
        inline bool operator()
        (const ResourceCont& lhs,
         const ResourceCont& rhs) const
        {
          if (lhs.hop == max_hop && rhs.hop == max_hop) {
            return !(lhs.compare(rhs.weight, lhs.weight));
          }
          return false;
        }
      private:
        std::size_t max_hop;
      };

      class DagDominanceFunc
      {
      public:
        DagDominanceFunc(std::size_t = 0) { }
        inline bool operator()
        (const ResourceCont& lhs,
         const ResourceCont& rhs) const
        {
          if (lhs.hop == rhs.hop) {
            return !(lhs.compare(rhs.weight, lhs.weight));
          }
          return false;
        }
      };

      typedef std::vector<Edge> EdgeVector;
      typedef std::vector<EdgeVector> ParetoOptSols;
      typedef std::vector<ResourceCont> ParetoOptResConts;
    };

  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap, class Compare, class Combine, class DistZero,
            class DFunc>
    bool hop_constrained_shortest_paths_impl
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptSols&
     pareto_optimal_solutions,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptResConts&
     pareto_optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero, DFunc)
    {
      typedef HopConstrainedClass<Graph, WeightMap, Compare> HopCC;
      typedef typename HopCC::ResourceCont RCont;
      typedef typename HopCC::template ResourceExtensionFunc<Combine> REFunc;

      boost::r_c_shortest_paths(
          g, v_index, e_index, s, t,
          pareto_optimal_solutions,
          pareto_optimal_resource_containers,
          RCont(zero, 0, compare),
          REFunc(t, weight_map, hop, combine),
          DFunc(hop));
      return !(pareto_optimal_resource_containers.empty());
    }

  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap, class Compare, class Combine, class DistZero>
    bool hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptSols&
     pareto_optimal_solutions,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptResConts&
     pareto_optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero)
    {
      typedef HopConstrainedClass<Graph, WeightMap, Compare> HopCC;
      return hop_constrained_shortest_paths_impl(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_optimal_solutions, pareto_optimal_resource_containers,
          compare, combine, zero, typename HopCC::DominanceFunc());
    }

  template <class Graph, class VertexIndexMap,
            class EdgeIndexMap, class WeightMap>
    bool hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename HopConstrainedClass<Graph, WeightMap>::ParetoOptSols&
     pareto_optimal_solutions,
     typename HopConstrainedClass<Graph, WeightMap>::ParetoOptResConts&
     pareto_optimal_resource_containers)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_optimal_solutions, pareto_optimal_resource_containers,
          std::less<WeightValue>(),
          boost::closed_plus<WeightValue>(), WeightValue());
    }


  template <class Graph, class VertexIndexMap,
            class EdgeIndexMap, class WeightMap>
    bool hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map)
    {
      typedef HopConstrainedClass<Graph, WeightMap> HopCC;
      typename HopCC::ParetoOptSols pareto_opt_sols;
      typename HopCC::ParetoOptResConts pareto_opt_res_conts;
      return hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_opt_sols, pareto_opt_res_conts);
    }

  // directed acyclic graph version
  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap, class Compare, class Combine, class DistZero>
    bool dag_hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptSols&
     pareto_optimal_solutions,
     typename HopConstrainedClass<Graph, WeightMap, Compare>::ParetoOptResConts&
     pareto_optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero)
    {
      typedef HopConstrainedClass<Graph, WeightMap, Compare> HopCC;
      return hop_constrained_shortest_paths_impl(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_optimal_solutions, pareto_optimal_resource_containers,
          compare, combine, zero, typename HopCC::DagDominanceFunc());
    }

  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap>
    bool dag_hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename HopConstrainedClass<Graph, WeightMap>::ParetoOptSols&
     pareto_optimal_solutions,
     typename HopConstrainedClass<Graph, WeightMap>::ParetoOptResConts&
     pareto_optimal_resource_containers)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return dag_hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_optimal_solutions, pareto_optimal_resource_containers,
          std::less<WeightValue>(),
          boost::closed_plus<WeightValue>(), WeightValue());
    }

  template <class Graph, class VertexIndexMap,
            class EdgeIndexMap, class WeightMap>
    bool dag_hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map)
    {
      typedef HopConstrainedClass<Graph, WeightMap> HopCC;
      typename HopCC::ParetoOptSols pareto_opt_sols;
      typename HopCC::ParetoOptResConts pareto_opt_res_conts;
      return dag_hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          pareto_opt_sols, pareto_opt_res_conts);
    }
}

#endif  // CANARD_HOP_CONSTRAINED_SHORTEST_PATHS_HPP

