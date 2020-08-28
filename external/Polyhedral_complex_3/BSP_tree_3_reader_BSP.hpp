#ifndef POLYHEDRAL_COMPLEX_3_BSP_TREE_3_READER_BSP_HPP
#define POLYHEDRAL_COMPLEX_3_BSP_TREE_3_READER_BSP_HPP

#include<ios>
#include<iostream>
#include<stack>
#include<utility>

namespace Polyhedral_complex_3 {

template<class Tree>
class BSP_tree_3_reader_BSP {
    protected:
        typedef typename Tree::Kernel Kernel;
        typedef typename Tree::Node Node;
        typedef typename Tree::Node_handle Node_handle;
        typedef typename Tree::Partitioner Partitioner;
        typedef typename Tree::Partitioner_handle Partitioner_handle;
        typedef typename Tree::Plane Plane;

        typedef typename Kernel::FT FT;

    public:
        BSP_tree_3_reader_BSP(Tree& tree) : _tree(tree) { }

        void read(std::istream& stream)
        {
            typedef std::pair<Node_handle, int> Pair;

            _tree.clear();

            std::stack<Pair> S;

            const Node_handle root_nh = _tree.root_handle();
            S.push(Pair(root_nh, 0));

            std::string line_str;
            int line_num = 0;

            while (!S.empty()) {
                line_num++;

                if (std::getline(stream, line_str) == 0) {
                    if (!stream.eof()) {
                        std::cerr << "input error: can't read line "
                                  << line_num << std::endl;
                        stream.clear(std::ios::badbit);
                        return;
                    } else {
                        stream.clear(std::ios::goodbit);
                        break;
                    }
                }

                std::string::size_type pos = line_str.find_first_not_of(' '); 
                const int j = pos / 4;

                const Pair pair = S.top();
                S.pop();

                Node_handle nh = pair.first;
                int i = pair.second;

                /* Empty line: leaf node with a twin non-leaf, so just skip */
                if (pos == std::string::npos)
                    continue;

                const std::string substr = line_str.substr(pos);

                std::stringstream line_stream(substr);

                /* Try to parse a plane */
                const Plane pl = _read_plane(line_stream);

                if (line_stream == 0) {
                    std::cerr << "input error: can't parse line "
                              << line_num << std::endl;
                    stream.clear(std::ios::badbit);
                    return;
                }

                /*
                 * Different indentation, so remove twin leaf nodes until
                 * the indentation is the same.
                 */
                while (i != j) {
                    S.pop(); /* Remove the second twin leaf node */

                    const Pair pair = S.top();
                    S.pop();
                    nh = pair.first;
                    i = pair.second;
                }

                /* Create a new node */
                _tree.partition(nh, pl);

                const Node& n = _tree.node(nh);
                const Node_handle negative_nh = n.negative_node_handle();
                const Node_handle positive_nh = n.positive_node_handle();

                S.push(Pair(positive_nh, i + 1));
                S.push(Pair(negative_nh, i + 1));
            }
        }

    protected:
        static Plane _read_plane(std::istream& stream)
        {
            FT nx, ny, nz, d;

            stream >> nx >> ny >> nz >> d;

            if (stream != 0)
                return Plane(nx, ny, nz, d);

            stream.clear(std::ios::badbit);
            return Plane(0, 0, 0, 0);
        }

    protected:
        Tree& _tree;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_BSP_TREE_3_READER_BSP_HPP */
