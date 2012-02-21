#ifndef CANARD_MY_PLOD_HPP
#define CANARD_MY_PLOD_HPP

#include <vector>
#include <map>
#include <iterator>
#include <utility>
#include <boost/random/uniform_int.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/config/no_tr1/cmath.hpp>
#include <boost/mpl/if.hpp>

namespace Canard {
  using namespace boost;
  template<typename RandomGenerator>
  class undirected_plod_iterator
  {
    typedef std::vector<std::pair<std::size_t, std::size_t> > out_degrees_t;

  public:
    typedef std::input_iterator_tag              iterator_category;
    typedef std::pair<std::size_t, std::size_t>  value_type;
    typedef const value_type&                    reference;
    typedef const value_type*                    pointer;
    typedef std::ptrdiff_t                       difference_type;

    undirected_plod_iterator() 
      : gen(0), out_degrees(), degrees_left(0), allow_self_loops(false) { }

    undirected_plod_iterator(RandomGenerator& gen, std::size_t n,  
                             double alpha, double beta, 
                             bool allow_self_loops = false)
      : gen(&gen), n(n), out_degrees(new out_degrees_t),
        degrees_left(0), allow_self_loops(allow_self_loops)
    {
      using std::pow;

      uniform_int<std::size_t> x(0, n-1);
      for (std::size_t i = 0; i != n; ++i) {
        std::size_t xv = x(gen);
        std::size_t degree = (xv < 2? 0 : std::size_t(beta * pow(xv - 1, -alpha)));
        if (degree < 2) degree = 2;
        else if (degree >= n) degree = n-1;
        out_degrees->push_back(std::make_pair(i, degree));
        degrees_left += degree;
      }

      next();
    }

    reference operator*() const { return current; }
    pointer operator->() const { return &current; }

    undirected_plod_iterator& operator++()
    { 
      next();
      return *this;
    }

    undirected_plod_iterator operator++(int)
    {
      undirected_plod_iterator temp(*this);
      ++(*this);
      return temp;
    }

    bool operator==(const undirected_plod_iterator& other) const
    { 
      return degrees_left == other.degrees_left; 
    }

    bool operator!=(const undirected_plod_iterator& other) const
    { return !(*this == other); }

  private:
    void next()
    {
      std::size_t source, target;
      while (true) {
        /* We may get to the point where we can't actually find any
           new edges, so we just add some random edge and set the
           degrees left = 0 to signal termination. */
        if (out_degrees->size() < 2) {
          uniform_int<std::size_t> x(0, n-1);
          current.first  = x(*gen);
          do {
            current.second = x(*gen);
          } while (current.first == current.second && !allow_self_loops);
          degrees_left = 0;
          out_degrees->clear();
          return;
        }

        uniform_int<std::size_t> x(0, out_degrees->size()-1);

        // Select source vertex
        source = x(*gen);
        if ((*out_degrees)[source].second == 0) {
          (*out_degrees)[source] = out_degrees->back();
          out_degrees->pop_back();
          continue;
        } 

        // Select target vertex
        target = x(*gen);
        if ((*out_degrees)[target].second == 0) {
          (*out_degrees)[target] = out_degrees->back();
          out_degrees->pop_back();
          continue;
        } else if (source != target 
                   || (allow_self_loops && (*out_degrees)[source].second > 2)) {
          break;
        }
      }

      // Update degree counts
      --(*out_degrees)[source].second;
      --degrees_left;
      --(*out_degrees)[target].second;
      --degrees_left;
      current.first  = (*out_degrees)[source].first;
      current.second = (*out_degrees)[target].first;
    }

    RandomGenerator* gen;
    std::size_t n;
    shared_ptr<out_degrees_t> out_degrees;
    std::size_t degrees_left;
    bool allow_self_loops;
    value_type current;
  };
} // namespace Canard

#endif  // CANARD_MY_PLOD_HPP

