#ifndef CANARD_UNEVEN_DIST_GRAPH_INFO_H
#define CANARD_UNEVEN_DIST_GRAPH_INFO_H

#include "Canard/graph/hop_constrained_shortest_paths.hpp"

namespace Canard {
  namespace detail {
    template <class Graph, class WeightMap, class Compare, class Combine>
      struct graph_info {
        typedef typename boost::property_traits<WeightMap>::value_type
            WeightValue;
        typedef hop_constrained_class<Graph, WeightMap> HopConstClass;
        typedef typename HopConstClass::EdgeVector EdgeVector;
        typedef typename HopConstClass::OptimalSolutions OptimalSolutions;
        typedef typename HopConstClass::OptimalResouceContainers
          OptimalResouceContainers;

        template <class DistZero>
        graph_info
        (const Graph& g, Compare cmp, Combine& cmb, DistZero zero,
         OptimalSolutions& opt_sols, OptimalResouceContainers& opt_res_conts)
          : graph(g), num_center(0), prev_value(zero), next_value(zero),
          compare(cmp), combine(&cmb),
          optimal_solutions(&opt_sols),
          optimal_resource_containers(&opt_res_conts)
        {
          using boost::num_vertices;
          using boost::vertex;
          typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
          if (num_vertices(graph) > 1) {
            const Vertex s = vertex(0, graph);
            const Vertex t = vertex(num_vertices(graph) - 1, graph);
            typename boost::graph_traits<Graph>::edge_descriptor e;
            boost::tie(e, boost::tuples::ignore) = edge(s, t, graph);
            prev_value = g[e];
            prev_solution.push_back(e);
            calc_next_sols();
          }
        }

        bool is_allocable() const
        {
          using boost::num_vertices;
          return num_vertices(graph) > num_center + 2;
        }

        void update()
        {
          ++num_center;
          prev_value = next_value;
          prev_solution.swap(next_solution);
          calc_next_sols();
        }

        WeightValue get_value() const
        {
          return prev_value;
        }

        bool operator<(const graph_info& rhs) const
        {
          // prev_value - next_value < rhs.prev_value - rhs.next_value
          // prev_value + rhs.next_value < rhs.prev_value + next_value
          // value1 < value2
          const WeightValue value1 = (*combine)(prev_value, rhs.next_value);
          const WeightValue value2 = (*combine)(rhs.prev_value, next_value);
          if (compare(value1, value2)) {
            if (compare(value2, value1)) {
              return !(num_center < rhs.num_center);
            }
            return true;
          }
          return false;
        }

        void calc_next_sols()
        {
          if (is_allocable()) {
            dag_hop_constrained_shortest_paths(
                graph,
                num_center + 2,
                get(boost::vertex_index, graph),
                get(boost::edge_index, graph),
                vertex(0, graph),
                vertex(num_vertices(graph) - 1, graph),
                get(boost::edge_bundle, graph),
                *optimal_solutions,
                *optimal_resource_containers);

            next_value = (*optimal_resource_containers)[0].weight;
            next_solution.swap((*optimal_solutions)[0]);
          }
        }

        Graph graph;
        std::size_t num_center;
        WeightValue prev_value;
        WeightValue next_value;
        EdgeVector prev_solution;
        EdgeVector next_solution;
        Compare compare;
        Combine *combine;
        OptimalSolutions* optimal_solutions;
        OptimalResouceContainers* optimal_resource_containers;
      };
  }
}

#endif  // CANARD_UNEVEN_DIST_GRAPH_INFO_H

