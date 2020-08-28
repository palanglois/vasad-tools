#ifndef POLYHEDRAL_COMPLEX_3_PRINT_OFF_HPP
#define POLYHEDRAL_COMPLEX_3_PRINT_OFF_HPP

#include <Polyhedral_complex_3/Mesh_3_writer_OFF.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
void print_mesh_OFF(std::ostream& stream, const Mesh& mesh)
{
    Mesh_3_writer_OFF<Mesh> writer(mesh);

    writer.write(stream);
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_PRINT_OFF_HPP */
