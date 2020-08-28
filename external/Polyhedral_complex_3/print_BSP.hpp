#ifndef POLYHEDRAL_COMPLEX_3_PRINT_BSP_HPP
#define POLYHEDRAL_COMPLEX_3_PRINT_BSP_HPP

#include <Polyhedral_complex_3/BSP_tree_3_writer_BSP.hpp>

namespace Polyhedral_complex_3 {

template<class Tree>
void print_tree_BSP(std::ostream& stream, const Tree& tree)
{
    BSP_tree_3_writer_BSP<Tree> writer(tree);

    writer.write(stream);
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_PRINT_BSP_HPP */
