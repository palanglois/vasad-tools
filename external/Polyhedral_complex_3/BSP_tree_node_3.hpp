#ifndef POLYHEDRAL_COMPLEX_3_BSP_TREE_NODE_3_HPP
#define POLYHEDRAL_COMPLEX_3_BSP_TREE_NODE_3_HPP

#include<cassert>

namespace Polyhedral_complex_3 {

template<class Ph,
         class Nh>
class BSP_tree_node_3 {
    public:
        typedef Ph Partitioner_handle;
        typedef Nh Node_handle;

    public:
        BSP_tree_node_3() :
            _ph(),
            _parent_nh(),
            _children_nh() { }

        BSP_tree_node_3(Node_handle parent_nh) :
            _ph(),
            _parent_nh(parent_nh),
            _children_nh() { assert(parent_nh != Node_handle()); }

        /**************************************/

        Partitioner_handle partitioner_handle() const { return _ph; }

        Partitioner_handle& partitioner_handle() { return _ph; }

        /**************************************/

        Node_handle parent_node_handle() const { return _parent_nh; }

        Node_handle children_node_handle() const { return _children_nh; }

        Node_handle& children_node_handle() { return _children_nh; }

        Node_handle child_node_handle(int i) const
        {
            assert(i >= 0 && i < 2);

            return (children_node_handle() + i);
        }

        /**************************************/

        Node_handle negative_node_handle() const { return child_node_handle(0); }

        Node_handle positive_node_handle() const { return child_node_handle(1); }

        /**************************************/

        bool is_root() const { return (parent_node_handle() == Node_handle()); }

        bool is_leaf() const { return (children_node_handle() == Node_handle()); }

    protected:
        Partitioner_handle _ph;
        Node_handle _parent_nh;
        Node_handle _children_nh; /* Node handle of the first child */
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_BSP_TREE_NODE_3_HPP */
