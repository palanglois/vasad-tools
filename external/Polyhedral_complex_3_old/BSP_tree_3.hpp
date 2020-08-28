#ifndef POLYHEDRAL_COMPLEX_3_BSP_TREE_3_HPP
#define POLYHEDRAL_COMPLEX_3_BSP_TREE_3_HPP

#include<cassert>
#include<queue>
#include<vector>

#include <CGAL/basic.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <Polyhedral_complex_3/BSP_tree_node_3.hpp>

namespace Polyhedral_complex_3 {

template<class K = CGAL::Exact_predicates_exact_constructions_kernel>
class BSP_tree_3 {
    public:
        typedef BSP_tree_3<K> Self;

        typedef K Kernel;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Point_3 Point;
        typedef typename Kernel::Vector_3 Vector;

        typedef Plane Partitioner;

    public:
        struct Partitioner_handle {
            public:
                Partitioner_handle() : _i(-1) { }

                Partitioner_handle(int i) : _i(i) { }

                bool operator==(Partitioner_handle ph) const { return (_i == ph._i); }

                operator int() const { return _i; }

            protected:
                int _i;
        };

        struct Node_handle {
            public:
                Node_handle() : _i(-1) { }

                Node_handle(int i) : _i(i) { }

                bool operator==(Node_handle nh) const { return (_i == nh._i); }

                operator int() const { return _i; }

            protected:
                int _i;
        };

    public:
        typedef BSP_tree_node_3<Partitioner_handle,
                                Node_handle> Node;

    protected:
        typedef std::vector<Partitioner> Partitioner_list;
        typedef std::vector<Node> Node_list;

    public:
        typedef typename Node_list::const_iterator
            Nodes_const_iterator;

    protected:
        enum { _dimension = 3 };
        enum { _number_of_children = 2 };

    public:
        BSP_tree_3() { _create_root(); }

        /*! Returns the (hardcoded) dimension of the BSP tree. */
        static int dimension() { return _dimension; }

        /*! Returns the (hardcoded) number of children of each node. */
        static int number_of_children() { return _number_of_children; }

        /*! Clears the BSP tree. */
        void clear()
        {
            _clear_partitioners();
            _clear_nodes();
        }

        /* Nodes ******************************/

        Node_handle root_handle() const { return 0; }

        const Node& root() const { return node(root_handle()); }

        const Node& node(Node_handle nh) const { return _nodes[nh]; }

        Node_handle node_handle(const Node& n) const { return (&n - &_nodes[0]); }

        Nodes_const_iterator nodes_begin() const { return _nodes.begin(); }

        Nodes_const_iterator nodes_end() const { return _nodes.end(); }

        /*! Returns the number of leaves. */
        int number_of_leaves() const { return ((number_of_nodes() / number_of_children()) + 1); }

      //int number_of_leaves() const { return (number_of_nodes()
      //                                     - number_of_partitioners()); }

        /*! Returns the total number of nodes. */
        int number_of_nodes() const { return _nodes.size(); }

        bool is_root(const Node& n) const { return is_root(node_handle(n)); }

      //bool is_root(const Node& n) const { return n.is_root(); }

        bool is_root(Node_handle nh) const { return (nh == root_handle()); }

        bool is_leaf(const Node& n) const { return n.is_leaf(); }

        bool is_leaf(Node_handle nh) const { return is_leaf(node(nh)); }

        /* Partitioners ***********************/

        int number_of_partitioners() const { return _partitioners.size(); }

        const Partitioner& partitioner(Partitioner_handle ph) const { return _partitioners[ph]; }

        bool partition(const Node& n, const Plane& pl) { return partition(node_handle(n), pl); }

        /*!
         *  Partitions a leaf node with the given plane.
         *  \param nh handle to the leaf node to be partitioned
         *  \param pl the partitioning plane
         *  \return true if the node was successfully split
         */
        bool partition(Node_handle nh, const Plane& pl)
        {
            const Node& n = node(nh);

            if (!n.is_leaf())
                return false;

            /* Nodes ******************************/
            const int number_of_nodes = this->number_of_nodes();
            _nodes.reserve(number_of_nodes + number_of_children());

            const Node_handle children_nh = Node_handle(number_of_nodes);

            _nodes.push_back(Node(nh)); /* negative_nh = children_nh */
            _nodes.push_back(Node(nh)); /* positive_nh = children_nh + 1 */

            /* Partitioner ************************/
            const int number_of_partitioners = this->number_of_partitioners();
            _partitioners.reserve(number_of_partitioners + 1);

            const Partitioner_handle ph =
                Partitioner_handle(number_of_partitioners);

            _partitioners.push_back(pl);

            /* Node update ************************/
            {
                Node& n = _node(nh);

                n.partitioner_handle() = ph;
                n.children_node_handle() = children_nh;
            }

            return true;
        }

        /*! Returns the node where the point is located. */
        Node_handle locate(const Point& p) const
        {
            Node_handle nh = root_handle();

            while (!is_leaf(nh)) {
                const Node& n = node(nh);

                const Partitioner_handle ph = n.partitioner_handle();

                const int p_side = partitioner(ph).oriented_side(p);

                if (p_side >= 0) {
                    const Node_handle positive_nh = n.positive_node_handle();
                    nh = positive_nh;
                } else {
                    const Node_handle negative_nh = n.negative_node_handle();
                    nh = negative_nh;
                }
            }

            return nh;
        }

        /* LCA ********************************/

        int compute_depth(Node_handle nh) const
        {
            int depth = 0;

            while (nh != root_handle()) {
                const Node& n = node(nh);
                const Node_handle parent_nh = n.parent_node_handle();
                nh = parent_nh;

                ++depth;
            }

            return depth;
        }

        Node_handle find_lowest_common_ancestor(Node_handle nh1,
                                                Node_handle nh2) const
        {
            assert(nh1 != Node_handle());
            assert(nh2 != Node_handle());

            int depth1 = compute_depth(nh1);
            int depth2 = compute_depth(nh2);

            while (depth1 > depth2) {
                nh1 = node(nh1).parent_node_handle();
                --depth1;
            }

            while (depth2 > depth1) {
                nh2 = node(nh2).parent_node_handle();
                --depth2;
            }

            while (nh1 != nh2) {
                nh1 = node(nh1).parent_node_handle();
                nh2 = node(nh2).parent_node_handle();
            }

            return nh1;
        }

        /*!
         *  Checks whether the given node is a left descendant of some other
         *  node.
         */
        bool is_negative_descendant(Node_handle nh,
                                    Node_handle ancestor_nh) const
        {
            while (nh != root_handle()) {
                const Node& n = node(nh);
                const Node_handle parent_nh = n.parent_node_handle();

                if (parent_nh != ancestor_nh) {
                    nh = parent_nh;

                    continue;
                }

                /* parent_nh == ancestor_nh */
                const Node& parent_n = node(parent_nh);

                assert(nh == parent_n.negative_node_handle()
                    || nh == parent_n.positive_node_handle());

                return (nh == parent_n.negative_node_handle());
            }

            return false;
        }

        bool is_positive_descendant(Node_handle nh,
                                    Node_handle ancestor_nh) const { return !is_negative_descendant(nh, ancestor_nh); }

        /*!
         *  Assigns a unique id to each partitioner that only depends on
         *  the tree structure.
         */
        template<class RandomAccessContainer>
        void order_partitioners(RandomAccessContainer& ids) const
        {
            const Node_handle root_nh = root_handle();
            const Node& root_n = node(root_nh);

            std::queue<Node_handle> Q;

            if (!root_n.is_leaf())
                Q.push(root_nh);

            int id = 0;

            while (!Q.empty()) {
                const Node_handle nh = Q.front();
                Q.pop();

                const Node& n = node(nh);
                const Partitioner_handle ph = n.partitioner_handle();
                assert(ph != Partitioner_handle());

                ids[ph] = id;
                ++id;

                const Node_handle negative_nh = n.negative_node_handle();
                const Node& negative_n = node(negative_nh);

                if (!negative_n.is_leaf())
                    Q.push(negative_nh);

                const Node_handle positive_nh = n.positive_node_handle();
                const Node& positive_n = node(positive_nh);

                if (!positive_n.is_leaf())
                    Q.push(positive_nh);
            }
        }

    protected:
        Node& _node(Node_handle nh) { return _nodes[nh]; }

        Node& _root() { return _node(root_handle()); }

        /*! Creates an empty BSP tree with only one leaf (the whole space). */
        void _create_root() { _nodes.push_back(Node()); }

        void _clear_partitioners() { _partitioners.clear(); }

        void _clear_nodes()
        {
            _nodes.resize(1);

            Node& root_n = _root();
            root_n.partitioner_handle() = Partitioner_handle();
            root_n.children_node_handle() = Node_handle();
        }

    protected:
        Partitioner_list _partitioners;
        Node_list _nodes;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_BSP_TREE_3_HPP */
