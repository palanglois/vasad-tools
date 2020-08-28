#ifndef POLYHEDRAL_COMPLEX_3_PRINT_POLY_HPP
#define POLYHEDRAL_COMPLEX_3_PRINT_POLY_HPP

#include <Polyhedral_complex_3/Mesh_3_writer_POLY.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
void print_mesh_POLY(std::ostream& stream,
                     const Mesh& mesh)
{
    Mesh_3_writer_POLY<Mesh> writer(mesh);

    writer.write(stream);
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_PRINT_POLY_HPP */
