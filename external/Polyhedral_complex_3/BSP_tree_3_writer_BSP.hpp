#ifndef POLYHEDRAL_COMPLEX_3_BSP_TREE_3_WRITER_BSP_HPP
#define POLYHEDRAL_COMPLEX_3_BSP_TREE_3_WRITER_BSP_HPP

#include<iostream>
#include<stack>
#include<utility>

namespace Polyhedral_complex_3 {

template<class Tree>
class BSP_tree_3_writer_BSP {
    protected:
        typedef typename Tree::Node Node;
        typedef typename Tree::Node_handle Node_handle;
        typedef typename Tree::Partitioner Partitioner;
        typedef typename Tree::Partitioner_handle Partitioner_handle;
        typedef typename Tree::Plane Plane;

    public:
        BSP_tree_3_writer_BSP(const Tree& tree) : _tree(tree) { }

        void write(std::ostream& stream) const
        {
            typedef std::pair<Node_handle, int> Pair;

            std::stack<Pair> S;

            const Node_handle root_nh = _tree.root_handle();
            S.push(Pair(root_nh, 0));

            while (!S.empty()) {
                const Pair pair = S.top();
                S.pop();

                const Node_handle nh = pair.first;
                const int i = pair.second;

                const Node& n = _tree.node(nh);

                int j = 4 * i;

                while (j--)
                    stream << ' ';

                if (n.is_leaf()) {
                    stream << std::endl;
                    continue;
                }

                const Partitioner_handle ph = n.partitioner_handle();
                const Plane& pl = _tree.partitioner(ph);

                stream << pl.a() << ' '
                       << pl.b() << ' '
                       << pl.c() << ' '
                       << pl.d() << std::endl;

                const Node_handle positive_nh = n.positive_node_handle();
                const Node_handle negative_nh = n.negative_node_handle();

                if (!_tree.node(positive_nh).is_leaf()
                 || !_tree.node(negative_nh).is_leaf()) {
                    S.push(Pair(positive_nh, i + 1));
                    S.push(Pair(negative_nh, i + 1));
                }
            }
        }

    protected:
        const Tree& _tree;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_BSP_TREE_3_WRITER_BSP_HPP */
