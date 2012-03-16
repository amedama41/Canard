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
    struct hop_constrained_class {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;

      struct resource_container
      {
        WeightValue weight;
        std::size_t hop;
        Compare compare;

        resource_container
        (const WeightValue w = WeightValue(),
         std::size_t h = 0, const Compare c = Compare())
          : weight(w), hop(h), compare(c) { }

        bool operator<(const resource_container& rhs) const
        {
          return (hop == rhs.hop) ? compare(weight, rhs.weight) : false;
        }

        bool operator==(const resource_container& rhs) const
        {
          return (hop == rhs.hop) &&
            (!compare(weight, rhs.weight) && !compare(rhs.weight, weight));
        }
      };

      template <class Combine>
      class resource_extension_functor
      {
      public:
        resource_extension_functor
        (Vertex t, WeightMap w, std::size_t h, Combine c)
          : target_v_(t), w_map_(w), max_hop_(h), combine_(c) { }

        inline bool operator()
        (const Graph& g, resource_container& new_cont,
         const resource_container& old_cont, Edge e) const
        {
          new_cont.weight = combine_(old_cont.weight, get(w_map_, e));
          new_cont.hop = old_cont.hop + 1;

          if (target(e, g) == target_v_) {
            return new_cont.hop == max_hop_;
          }
          return (new_cont.hop < max_hop_);
        }

      private:
        Vertex target_v_;
        WeightMap w_map_;
        std::size_t max_hop_;
        Combine combine_;
      };

      class dominance_functor
      {
      public:
        dominance_functor(std::size_t h = 0) : max_hop_(h) { }

        inline bool operator()
        (const resource_container& lhs,
         const resource_container& rhs) const
        {
          if (lhs.hop == max_hop_ && rhs.hop == max_hop_) {
            return !(lhs.compare(rhs.weight, lhs.weight));
          }
          return false;
        }

      private:
        std::size_t max_hop_;
      };

      class dag_dominance_functor
      {
      public:
        dag_dominance_functor(std::size_t = 0) { }

        inline bool operator()
        (const resource_container& lhs,
         const resource_container& rhs) const
        {
          if (lhs.hop == rhs.hop) {
            return !(lhs.compare(rhs.weight, lhs.weight));
          }
          return false;
        }
      };

      typedef std::vector<Edge> EdgeVector;
      typedef std::vector<EdgeVector> OptimalSolutions;
      typedef std::vector<resource_container> OptimalResouceContainers;
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
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalSolutions& optimal_solutions,
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalResouceContainers& optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero, DFunc)
    {
      typedef hop_constrained_class<Graph, WeightMap, Compare> HCClass;
      typedef typename HCClass::resource_container RCont;
      typedef typename HCClass::template resource_extension_functor<Combine>
        REFunc;

      boost::r_c_shortest_paths(
          g, v_index, e_index, s, t,
          optimal_solutions,
          optimal_resource_containers,
          RCont(zero, 0, compare),
          REFunc(t, weight_map, hop, combine),
          DFunc(hop));
      return !(optimal_resource_containers.empty());
    }

  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap, class Compare, class Combine, class DistZero>
    bool hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalSolutions& optimal_solutions,
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalResouceContainers& optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero)
    {
      typedef hop_constrained_class<Graph, WeightMap, Compare> HCClass;
      return hop_constrained_shortest_paths_impl(
          g, hop, v_index, e_index, s, t, weight_map,
          optimal_solutions, optimal_resource_containers,
          compare, combine, zero, typename HCClass::dominance_functor());
    }

  template <class Graph, class VertexIndexMap,
            class EdgeIndexMap, class WeightMap>
    bool hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename hop_constrained_class<Graph, WeightMap>::OptimalSolutions&
       optimal_solutions,
     typename hop_constrained_class<Graph, WeightMap>::OptimalResouceContainers&
       optimal_resource_containers)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          optimal_solutions, optimal_resource_containers,
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
      typedef hop_constrained_class<Graph, WeightMap> HCClass;
      typename HCClass::OptimalSolutions opt_sols;
      typename HCClass::OptimalResouceContainers opt_res_conts;
      return hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          opt_sols, opt_res_conts);
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
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalSolutions& optimal_solutions,
     typename hop_constrained_class<
         Graph, WeightMap, Compare
       >::OptimalResouceContainers& optimal_resource_containers,
     Compare compare, Combine combine, DistZero zero)
    {
      typedef hop_constrained_class<Graph, WeightMap, Compare> HCClass;
      return hop_constrained_shortest_paths_impl(
          g, hop, v_index, e_index, s, t, weight_map,
          optimal_solutions, optimal_resource_containers,
          compare, combine, zero, typename HCClass::dag_dominance_functor());
    }

  template <class Graph, class VertexIndexMap, class EdgeIndexMap,
            class WeightMap>
    bool dag_hop_constrained_shortest_paths
    (const Graph& g, std::size_t hop,
     VertexIndexMap v_index, EdgeIndexMap e_index,
     typename boost::graph_traits<Graph>::vertex_descriptor s,
     typename boost::graph_traits<Graph>::vertex_descriptor t,
     WeightMap weight_map,
     typename hop_constrained_class<Graph, WeightMap>::OptimalSolutions&
       optimal_solutions,
     typename hop_constrained_class<Graph, WeightMap>::OptimalResouceContainers&
       optimal_resource_containers)
    {
      typedef
        typename boost::property_traits<WeightMap>::value_type
        WeightValue;
      return dag_hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          optimal_solutions, optimal_resource_containers,
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
      typedef hop_constrained_class<Graph, WeightMap> HCClass;
      typename HCClass::OptimalSolutions opt_sols;
      typename HCClass::OptimalResouceContainers opt_res_conts;
      return dag_hop_constrained_shortest_paths(
          g, hop, v_index, e_index, s, t, weight_map,
          opt_sols, opt_res_conts);
    }
}

#endif  // CANARD_HOP_CONSTRAINED_SHORTEST_PATHS_HPP

