#ifndef CANARD_ALTERNATE_P_CENTER
#define CANARD_ALTERNATE_P_CENTER

#include <vector>
#include <limits>
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/property_map_iterator.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include "Canard/graph/p_center_base.hpp"
#include "Canard/graph/random_p_center.hpp"
#include "Canard/property_map/property_map_copy.hpp"

namespace Canard {
  namespace detail {
    template <class DistanceMatrix, class IndexMap, class Compare>
      class DistCompare
      {
        public:
          typedef bool result_type;
          typedef typename IndexMap::key_type V;
          DistCompare(const DistanceMatrix& d, IndexMap i, Compare cmp)
            : d_matrix(&d), i_map(i), compare(cmp) { }
          bool operator()(V lhs1, V lhs2, V rhs1, V rhs2) const
          {
            return compare(
                (*d_matrix)[get(i_map, lhs1)][get(i_map, lhs2)],
                (*d_matrix)[get(i_map, rhs1)][get(i_map, rhs2)]);
          }
        private:
          const DistanceMatrix* d_matrix;
          IndexMap i_map;
          Compare compare;
      };

    template <class DemandMap, class GeneratorMap>
      class IsDemandAndHaveSameGenerator
      : public std::unary_function<typename DemandMap::key_type, bool>
      {
        public:
          typedef typename DemandMap::key_type Vertex;
          IsDemandAndHaveSameGenerator
            (DemandMap d_map, GeneratorMap g_map, Vertex g)
            : d_map(d_map), g_map(g_map), generator(g) { }
          bool operator()(Vertex v) const
          {
            return get(d_map, v) && get(g_map, v) == generator;
          }
        private:
          DemandMap d_map;
          GeneratorMap g_map;
          Vertex generator;
      };

    template <class DistValue, class Graph,
              class GeneratorMap, class MaxDistMap,
              class Centers, class DistanceMatrix,
              class IndexMap, class DemandMap, class Compare, class DistZero>
      void set_generator_and_max_dist
      (const Graph& g, GeneratorMap generator_map, MaxDistMap max_dist_map,
       const Centers& centers, const DistanceMatrix& d_matrix,
       IndexMap i_map, DemandMap demand_map, Compare compare, DistZero zero)
      {
        using boost::vertices;

        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
        typedef std::pair<VIter, VIter> VIterPair;
        DistCompare<DistanceMatrix, IndexMap, Compare>
          dist_compare(d_matrix, i_map, compare);
        boost::for_each(centers,
            make_property_map_copy(
              max_dist_map,
              boost::make_constant_property<Vertex>(zero)));
        for (VIterPair vi = vertices(g); vi.first != vi.second; ++vi.first) {
          const Vertex v = *vi.first;
          const Vertex closest_center = *boost::min_element(
              centers,
              boost::lambda::bind(
                dist_compare, v, boost::lambda::_1, v, boost::lambda::_2));
          put(generator_map, v, closest_center);

          if (get(demand_map, v)) {
            const DistValue min_dist
              = d_matrix[get(i_map, v)][get(i_map, closest_center)];
            if (compare(get(max_dist_map, closest_center), min_dist)) {
              put(max_dist_map, closest_center, min_dist);
            }
          }
        } // for
      } // set_generator_and_max_dist

    template <class Graph, class Vertex,
              class GeneratorMap, class DemandMap, class DistCompare>
      Vertex get_farthest_vertex
      (const Graph& g, Vertex center, GeneratorMap generator_map,
       DemandMap demand_map, DistCompare dist_compare)
      {
        using boost::vertices;
        typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
        const std::pair<VIter, VIter> vi = vertices(g);
        const IsDemandAndHaveSameGenerator<DemandMap, GeneratorMap>
          is_demand(demand_map, generator_map, get(generator_map, center));
        const VIter max_vi = std::max_element(
            boost::make_filter_iterator(is_demand, vi.first,  vi.second),
            boost::make_filter_iterator(is_demand, vi.second, vi.second),
            boost::lambda::bind(dist_compare,
              boost::lambda::_1, center, boost::lambda::_2, center)).base();
        return max_vi != vi.second ? *max_vi : center;
      } // get_farthest_vertex
  } // namespace detail

  template <class DistValue, class Graph, class CenterMap, class DistanceMatrix,
            class IndexMap, class CandidateMap, class DemandMap,
            class Compare, class DistInf, class DistZero>
    DistValue alternate_p_center
    (const Graph& g, CenterMap center_map, const DistanceMatrix& d_matrix,
     IndexMap i_map, CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, DistInf inf, DistZero zero)
    {
      using boost::vertices;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::vertex_iterator VIter;
      typedef std::pair<VIter, VIter> VIterPair;


      // init center vector
      const VIterPair vipair = vertices(g);
      const boost::property_map_function<CenterMap> map_func(center_map);
      std::vector<Vertex> centers
        (boost::make_filter_iterator(map_func, vipair.first,  vipair.second),
         boost::make_filter_iterator(map_func, vipair.second, vipair.second));

      // create generator_map
      typedef boost::iterator_property_map<
        typename std::vector<Vertex>::iterator, IndexMap>
        GeneratorMap;
      std::vector<Vertex> generator(num_vertices(g));
      GeneratorMap generator_map(generator.begin(), i_map);
      std::vector<Vertex> new_center(num_vertices(g));
      GeneratorMap new_center_map(new_center.begin(), i_map);

      // create max_dist_map
      typedef boost::iterator_property_map<
        typename std::vector<DistValue>::iterator, IndexMap>
        MaxDistMap;
      std::vector<DistValue> max_dist(num_vertices(g), zero);
      MaxDistMap max_dist_map(max_dist.begin(), i_map);

      detail::DistCompare<DistanceMatrix, IndexMap, Compare>
        dist_compare(d_matrix, i_map, compare);

      bool repeat = true;
      while (repeat) {
        repeat = false;
        typename std::vector<Vertex>::iterator
          ci = centers.begin(), ci_end = centers.end();
        // initialize generator
        std::copy(ci, ci_end,
            boost::make_property_map_iterator(generator_map, ci));
        // initialize max_dist
        std::fill(
            boost::make_property_map_iterator(max_dist_map, ci),
            boost::make_property_map_iterator(max_dist_map, ci_end),
            zero);
        // initialize new_center
        std::copy(ci, ci_end,
            boost::make_property_map_iterator(new_center_map, ci));

        // Network Voronoi Division
        detail::set_generator_and_max_dist<DistValue>(
            g, generator_map, max_dist_map,
            centers, d_matrix, i_map, demand_map, compare, zero);

        // Find new center
        for (VIterPair vi = vertices(g); vi.first != vi.second; ++vi.first) {
          if (get(candidate_map, *vi.first)) {
            const Vertex new_center = *vi.first;
            const Vertex farthest_vertex = detail::get_farthest_vertex(
                g, new_center, generator_map, demand_map, dist_compare);
            const DistValue dist
              = d_matrix[get(i_map, farthest_vertex)][get(i_map, new_center)];
            const Vertex old_center = get(generator_map, new_center);
            if (compare(dist, get(max_dist_map, old_center))) {
              put(new_center_map, old_center, new_center);
              put(max_dist_map, old_center, dist);
            }
          }
        }

        // Set new center
        for (ci = centers.begin(), ci_end = centers.end(); ci != ci_end; ++ci) {
          const Vertex new_center = get(new_center_map, *ci);
          if (*ci != new_center) {
            repeat = true;
            *ci = new_center;
          }
        }
      } // while

      // Set center_map
      std::for_each(vipair.first, vipair.second,
          make_property_map_copy(
            center_map, boost::make_constant_property<Vertex>(false)));
      std::for_each(centers.begin(), centers.end(),
          make_property_map_copy(
            center_map, boost::make_constant_property<Vertex>(true)));

      return *std::max_element(
          boost::make_property_map_iterator(max_dist_map, centers.begin()),
          boost::make_property_map_iterator(max_dist_map, centers.end()),
          compare);
    } // alternate_p_center

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap,
            class Compare, class Combine, class DistInf, class DistZero>
    typename boost::property_traits<WeightMap>::value_type
    alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero)
    {
      if (p == 0) {
        throw std::runtime_error("Number of center is 0"__FILE__);
      }
      using boost::num_vertices;
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      const typename DistMatrix<DistValue>::type d_matrix
        = create_distance_matrix(g, w_map, i_map, compare, combine, inf, zero);

      std::vector<char> temp_center(num_vertices(g));
      boost::iterator_property_map<std::vector<char>::iterator, IndexMap>
        temp_center_map(temp_center.begin(), i_map);
      DistValue opt_dist = inf;

      for (std::size_t i = 0; i < times; ++i) {
        random_p_center(g, p, 1, temp_center_map, w_map, i_map,
            candidate_map, demand_map, compare, combine, inf, zero);
        const DistValue dist = alternate_p_center<DistValue>
          (g, temp_center_map, d_matrix, i_map,
           candidate_map, demand_map, compare, inf, zero);

        if (compare(dist, opt_dist)) {
          boost::for_each(vertices(g),
              make_property_map_copy(center_map, temp_center_map));
          opt_dist = dist;
        }
      }
      return opt_dist;
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap, class DemandMap>
    typename boost::property_traits<WeightMap>::value_type
    alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map, DemandMap demand_map)
    {
      typedef typename boost::property_traits<WeightMap>::value_type DistValue;
      return alternate_p_center
        (g, p, times, center_map, w_map, i_map,
          candidate_map, demand_map,
         std::less<DistValue>(), boost::closed_plus<DistValue>(),
         std::numeric_limits<DistValue>::max(), DistValue());
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap,
            class CandidateMap>
    typename boost::property_traits<WeightMap>::value_type
    alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map,
     CandidateMap candidate_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return alternate_p_center
        (g, p, times, center_map, w_map, i_map, candidate_map,
         boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap, class IndexMap>
    typename boost::property_traits<WeightMap>::value_type
    alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times, CenterMap center_map,
     WeightMap w_map, IndexMap i_map)
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      return alternate_p_center
        (g, p, times, center_map, w_map, i_map,
         boost::make_constant_property<Vertex>(true));
    }

  template <class Graph, class CenterMap, class WeightMap>
    typename boost::property_traits<WeightMap>::value_type
    alternate_p_center
    (const Graph& g, std::size_t p, std::size_t times,
     CenterMap center_map, WeightMap w_map)
    {
      return alternate_p_center
        (g, p, times, center_map, w_map, get(boost::vertex_index, g));
    }
} // namespace Canard

#endif  // CANARD_ALTERNATE_P_CENTER

