#ifndef CANARD_LINEAR_GRAPH_CONTAINER_H
#define CANARD_LINEAR_GRAPH_CONTAINER_H

#include <list>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/filtered_graph.hpp>

namespace Canard {
  namespace detail {
    template <class Set>
      class is_in_subset {
        public:
          is_in_subset() { }
          is_in_subset(Set& set) { m_set.swap(set); }

          bool operator()(const typename Set::value_type& x) const
          {
            return m_set.find(x) != m_set.end();
          }

        private:
          Set m_set;
      };
  }

  template <class Graph>
    struct LinearGraph {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename boost::graph_traits<Graph>::edge_descriptor   Edge;
      typedef detail::is_in_subset< std::set<Vertex> > VertexPred;
      typedef detail::is_in_subset< std::set<Edge> >   EdgePred;
      // TODO const boost::filtered_graph<const Graph, ...
      typedef boost::filtered_graph<const Graph, EdgePred, VertexPred> type;
    };

  template <class Graph,
            class Container = std::list<typename LinearGraph<Graph>::type> >
    struct linear_graphs {
      public:
        typedef typename Container::value_type        value_type;
        typedef typename Container::const_pointer     pointer;
        typedef typename Container::reference         reference;
        typedef typename Container::const_reference   const_reference;
        typedef typename Container::iterator          iterator;
        typedef typename Container::const_iterator    const_iterator;
        typedef typename Container::const_reverse_iterator
          const_reverse_iterator;
        typedef typename Container::reverse_iterator  reverse_iterator;
        typedef typename Container::size_type         size_type;
        typedef typename Container::difference_type   difference_type;
        typedef typename Container::allocator_type    allocator_type;

        linear_graphs()
          : container() { }

        explicit linear_graphs(const allocator_type& a)
          : container(a) { }

        explicit linear_graphs(size_type n)
          : container(n) { }

        linear_graphs(size_type n, const value_type& value,
            const allocator_type& a = allocator_type())
          : container(n, value, a) { }

        linear_graphs(const linear_graphs& lg)
          : container(lg.container) { }

#ifdef  __GXX_EXPERIMENTAL_CXX0X__
        linear_graphs(linear_graphs&& lg)
          : container(std::move(lg.container)) { }

        linear_graphs(std::initializer_list<value_type> l,
            const allocator_type& a = allocator_type())
          : container(l, a) { }
#endif

        template <typename InputIterator>
          linear_graphs(InputIterator first, InputIterator last,
              const allocator_type& a = allocator_type())
          : container(first, last, a) { }

        linear_graphs& operator=(const linear_graphs& lg)
        {
          container = lg.container;
          return *this;
        }

        bool operator==(const linear_graphs& lg)
        { return container == lg.container; }

        bool operator<(const linear_graphs& lg)
        { return container < lg.container; }

        allocator_type get_allocator() const
        { return container.get_allocator(); }

        iterator begin()
        { return container.begin(); }

        const_iterator begin() const
        { return container.begin(); }

        iterator end()
        { return container.end(); }

        const_iterator end() const
        { return container.end(); }

        reverse_iterator rbegin()
        { return container.rbegin(); }

        const_reverse_iterator rbegin() const
        { return container.rbegin(); }

        reverse_iterator rend()
        { return container.rend(); }

        const_reverse_iterator rend() const
        { return container.rend(); }

#ifdef  __GXX_EXPERIMENTAL_CXX0X__
        const_iterator cbegin() const
        { return container.cbegin(); }

        const_iterator cend() const
        { return container.cend(); }

        const_reverse_iterator crbegin() const
        { return container.crbegin(); }

        const_reverse_iterator crend() const
        { return container.crend(); }
#endif

        bool empty() const
        { return container.empty(); }

        size_type size() const
        { return container.size(); }

        size_type max_size() const
        { return container.max_size(); }

        void resize(size_type new_size)
        { return container.resize(new_size); }

        void resize(size_type new_size, const value_type& g)
        { return container.resize(new_size, g); }

        reference front()
        { return container.front(); }

        const_reference front() const
        { return container.front(); }

        reference back()
        { return container.back(); }

        const_reference back() const
        { return container.back(); }

        void push_back(const value_type& g)
        { container.push_back(g); }

#ifdef  __GXX_EXPERIMENTAL_CXX0X__
        void push_back(value_type&& g)
        { container.push_back(g); }

        // template <typename... Args>
          // void emplace_back(Args&&... args)
          // { container.emplace_back(args...); }
#endif
        void pop_back()
        { container.pop_back(); }

        void swap(linear_graphs& lg)
        { container.swap(lg.container); }

        void clear()
        { container.clear(); }

      private:
        Container container;
    };
}

#endif  // CANARD_LINEAR_GRAPH_CONTAINER_H

