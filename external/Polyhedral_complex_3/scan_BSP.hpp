#ifndef POLYHEDRAL_COMPLEX_3_SCAN_BSP_HPP
#define POLYHEDRAL_COMPLEX_3_SCAN_BSP_HPP

#include <Polyhedral_complex_3/BSP_tree_3_reader_BSP.hpp>

namespace Polyhedral_complex_3 {

template<class Tree>
void scan_tree_BSP(std::istream& stream, Tree& tree)
{
    BSP_tree_3_reader_BSP<Tree> reader(tree);

    reader.read(stream);
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_SCAN_BSP_HPP */
