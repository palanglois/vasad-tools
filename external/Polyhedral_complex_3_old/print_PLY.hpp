#ifndef POLYHEDRAL_COMPLEX_3_PRINT_PLY_HPP
#define POLYHEDRAL_COMPLEX_3_PRINT_PLY_HPP

#include <Polyhedral_complex_3/Mesh_3_writer_PLY.hpp>
#include <Polyhedral_complex_3/Mesh_with_facet_info_3_writer_PLY.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
void print_mesh_PLY(std::ostream& stream,
                    const Mesh& mesh)
{
    Mesh_3_writer_PLY<Mesh> writer(mesh);

    writer.write(stream);
}

template<class Mesh,
         class Colormap>
void print_mesh_with_facet_color_PLY(std::ostream& stream,
                                     const Mesh& mesh,
                                     const Colormap& colormap)
{
    Mesh_with_facet_info_3_writer_PLY<Mesh,
                                      Colormap> writer(mesh,
                                                       colormap);

    writer.write(stream);
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_PRINT_PLY_HPP */
