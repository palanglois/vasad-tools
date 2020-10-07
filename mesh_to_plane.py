import argparse
import random
import json
from os.path import join
import numpy as np
from numpy.linalg import norm

import pymesh


class PlaneClass:
    def __init__(self):
        self.inlier_index = []
        self.inlier_points = []
        self.structural_lines = []
        self.texturial_lines = []
        self._normal_accumulator = np.array([0., 0., 0.])
        self._average_point_accumulator = np.array([0., 0., 0.])
        self.triangle_area_accumulator = 0
        self.normal = np.array([0., 0., 0.])
        self.average_point = np.array([0., 0., 0.])

    def add_triangle(self, index, normal, average, triangle):
        """
        Add an inlier triangle to the plane structure.
        Maintains the normal, average and area accumulators up to date.
        :param index: index of the triangle in the global triangle set
        :param normal: normal to the triangle
        :param average: triangle's center of gravity
        :param triangle: 3*3 numpy array containing the triangle's points
        :return:
        """
        # Insert the triangle in the class variables
        triangle_area = 0.5 * norm(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]))
        self.inlier_index.append(index)

        # Checking normal orientation before inserting
        if np.dot(self.normal, normal) < 0:
            self._normal_accumulator -= normal * triangle_area
        else:
            self._normal_accumulator += normal * triangle_area

        self._average_point_accumulator += average * triangle_area
        self.triangle_area_accumulator += triangle_area

        # Update the class characteristics
        self.normal = self.get_normal()
        self.average_point = self.get_ref_point()

        # Insert the points
        for point in triangle:
            self.inlier_points.append(point)

    def get_normal(self):
        """
        Get the average normal of the plane (with triangle area weighting)
        :return: a dimension 3 numpy array
        """
        return self._normal_accumulator / (np.sqrt(np.dot(self._normal_accumulator, self._normal_accumulator)))

    def get_ref_point(self):
        """
        Get the average reference point of the plane (with triangle area weighting)
        :return:
        """
        return self._average_point_accumulator / self.triangle_area_accumulator

    def merge_with(self, other_class):
        """
        Merges the current class with an other class
        :param other_class: an other PlaneClass object
        :return:
        """

        # Merge the accumulators and databases
        self.inlier_index += other_class.inlier_index
        self.inlier_points += other_class.inlier_points
        if np.dot(self.normal, other_class.normal) < 0:
            self._normal_accumulator -= other_class.normal
        else:
            self._normal_accumulator += other_class.normal
        self._average_point_accumulator += other_class._average_point_accumulator
        self.triangle_area_accumulator += other_class.triangle_area_accumulator

        # Update the class characteristics
        self.normal = self.get_normal()
        self.average_point = self.get_ref_point()

    def eval_structural_lines(self, triangle_to_edge):
        """
        Compute structural lines (and texturial lines at the same time)
        :param triangle_to_edge: list linking the triangle index to its 3 edges
        :return:
        """
        # Removing double entries
        self.inlier_index = list(set(self.inlier_index))
        # Gathering all the edges from the current class
        all_edges = [x for i in self.inlier_index for x in triangle_to_edge[i]]
        # Edges that appear once are texturial lines. Edges that appear twice are structural lines.
        for edge in all_edges:
            if all_edges.count(edge) == 1:
                self.structural_lines.append(edge)
            elif all_edges.count(edge) == 2 and edge not in self.texturial_lines:
                self.texturial_lines.append(edge)


class ClassComputer:
    def __init__(self, file_path, epsilon_point, epsilon_normal, sigma_point, sigma_normal):
        self.mesh = pymesh.load_mesh(file_path)
        self.mesh.normals = pymesh.face_normals(self.mesh.vertices, self.mesh.faces)
        self.epsilon_point = epsilon_point
        self.epsilon_normal = epsilon_normal
        self.sigma_point = sigma_point
        self.sigma_normal = sigma_normal
        self.computed_classes = []
        self.triangle_to_edge = []
        self.edge_to_triangle = dict()

    def insert_segment(self, version_1, version_2, triangle_index):
        """
        Tries to insert two versions of the same segment by checking already existing version.
        Returns the version inserted.
        :param version_1: first version of the segment
        :param version_2: second version of the segment
        :param triangle_index: index of the triangle the segment belongs to
        :return: The version of the segment that was actually inserted
        """
        if version_1 in self.edge_to_triangle:
            self.edge_to_triangle[version_1].append(triangle_index)
            return version_1
        elif version_2 in self.edge_to_triangle:
            self.edge_to_triangle[version_2].append(triangle_index)
            return version_2
        else:
            self.edge_to_triangle[version_1] = [triangle_index]
            return version_1

    def compute_classes_from_file_by_growing(self):
        """
        Main function to cluster a set of triangles, uses the growing approach
        :return:
        """

        # Build adjacency graph
        print("Building adjacency graph")
        number_of_triangles = self.mesh.faces.shape[0]
        self.triangle_to_edge = [None] * number_of_triangles
        for i in range(number_of_triangles):
            seg_1a = (*self.mesh.vertices[self.mesh.faces[i][0], :], *self.mesh.vertices[self.mesh.faces[i][1], :])
            seg_1b = (*self.mesh.vertices[self.mesh.faces[i][1], :], *self.mesh.vertices[self.mesh.faces[i][0], :])
            seg_2a = (*self.mesh.vertices[self.mesh.faces[i][1], :], *self.mesh.vertices[self.mesh.faces[i][2], :])
            seg_2b = (*self.mesh.vertices[self.mesh.faces[i][2], :], *self.mesh.vertices[self.mesh.faces[i][1], :])
            seg_3a = (*self.mesh.vertices[self.mesh.faces[i][2], :], *self.mesh.vertices[self.mesh.faces[i][0], :])
            seg_3b = (*self.mesh.vertices[self.mesh.faces[i][0], :], *self.mesh.vertices[self.mesh.faces[i][2], :])

            # Insert the segments with invariance to orientation
            insert_1 = self.insert_segment(seg_1a, seg_1b, i)
            insert_2 = self.insert_segment(seg_2a, seg_2b, i)
            insert_3 = self.insert_segment(seg_3a, seg_3b, i)

            # Insert the triangle
            self.triangle_to_edge[i] = [insert_1, insert_2, insert_3]

        # Region growing
        print("Performing region growing")
        grown_classes = []
        is_triangle_added = np.asarray([False] * number_of_triangles)
        decrease_area_permutation = self.compute_triangle_order()
        global_it = 0
        while not np.all(is_triangle_added):
            # We check that the current triangle has not been visited yet
            triangle_it = decrease_area_permutation[global_it]
            if is_triangle_added[triangle_it]:
                global_it += 1
                continue
            is_triangle_added[triangle_it] = True
            # We initiate a new class with the current unseen triangle
            new_class = PlaneClass()
            cur_triangle = [self.mesh.vertices[self.mesh.faces[triangle_it][0]], self.mesh.vertices[self.mesh.faces[triangle_it][1]], self.mesh.vertices[self.mesh.faces[triangle_it][2]]]
            average_point = sum(cur_triangle) / 3.
            new_class.add_triangle(triangle_it, self.mesh.normals[triangle_it], average_point, cur_triangle)
            # Compute the none visited neighbours of current triangle
            neighbour_list = [tri for edge in self.triangle_to_edge[triangle_it] for tri in self.edge_to_triangle[edge]
                              if tri != triangle_it and not is_triangle_added[tri]]
            # We propagate to the neighbours
            while len(neighbour_list) != 0:
                next_tri_id = neighbour_list.pop()
                next_triangle = [self.mesh.vertices[self.mesh.faces[next_tri_id][0]], self.mesh.vertices[self.mesh.faces[next_tri_id][1]], self.mesh.vertices[self.mesh.faces[next_tri_id][2]]]
                next_triangle_average = sum(next_triangle) / 3.
                if self.is_triangle_in_class(next_triangle_average, self.mesh.normals[next_tri_id], new_class):
                    is_triangle_added[next_tri_id] = True
                    new_class.add_triangle(next_tri_id, self.mesh.normals[next_tri_id],
                                           next_triangle_average, next_triangle)
                    # If we have an inlier, we test its neighbours
                    neighbour_list += [tri for edge in self.triangle_to_edge[next_tri_id] for tri in
                                       self.edge_to_triangle[edge]
                                       if tri != next_tri_id and not is_triangle_added[tri]]
            grown_classes.append(new_class)

        # Primitive merging
        print("Merging the primitives")
        # Sort classes by decreasing area
        grown_classes = sorted(grown_classes, key=lambda x: x.triangle_area_accumulator, reverse=True)
        number_of_grown_primitives = len(grown_classes)
        for i in range(number_of_grown_primitives):
            if i % (number_of_grown_primitives // 10) == 0:
                print("Progress : %s%%" % str(round(100. * float(i) / number_of_grown_primitives)))
            class_found = False
            for cur_class in self.computed_classes:
                if self.are_classes_same(cur_class, grown_classes[i]):
                    class_found = True
                    cur_class.merge_with(grown_classes[i])
                    break
            if not class_found:
                self.computed_classes.append(grown_classes[i])
        print("Number of primitive found : %s" % str(len(self.computed_classes)))

    def compute_triangle_order(self):
        """
        Computes the permutation that yields the triangles in a decreasing area order
        :return: an int array containing the permutation
        """
        number_of_triangles = self.mesh.faces.shape[0]
        all_triangles = [(self.mesh.vertices[self.mesh.faces[i][0]], self.mesh.vertices[self.mesh.faces[i][1]], self.mesh.vertices[self.mesh.faces[i][2]]) for i in range(number_of_triangles)]
        all_areas = [0.5 * norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) for tri in all_triangles]
        return sorted(range(len(all_areas)), key=lambda k: all_areas[k], reverse=True)

    def is_triangle_in_class(self, average_point, normal, plane_class):
        """
        Check that a given triangle is in a given class
        :param average_point: the triangle's gravity point
        :param normal: the triangle's normal
        :param plane_class: PlaneClass object that represents the tested plane class
        :return: True if the triangle is in plane_class, False otherwise
        """
        class_normal = plane_class.normal
        class_ref_point = plane_class.average_point
        point_contribution = abs(np.dot(average_point - class_ref_point,
                                        class_normal)) < self.epsilon_point
        point_contribution_2 = abs(np.dot(average_point - class_ref_point,
                                          normal)) < self.epsilon_point
        normal_contribution = 1 - abs(np.dot(normal, class_normal)) < self.epsilon_normal
        return point_contribution and normal_contribution and point_contribution_2

    def are_classes_same(self, class_1, class_2):
        """
        Function in order to test whether two PlaneClass object should be matched or not
        :param class_1: first PlaneClass object
        :param class_2: second PlaneClass object
        :return: a boolean
        """
        point_contribution_1 = abs(np.dot(class_1.average_point - class_2.average_point, class_1.normal)) \
                               < self.sigma_point
        point_contribution_2 = abs(np.dot(class_1.average_point - class_2.average_point, class_2.normal)) \
                               < self.sigma_point
        normal_contribution = 1 - abs(np.dot(class_1.normal, class_2.normal)) < self.sigma_normal
        return point_contribution_1 and point_contribution_2 and normal_contribution

    def compute_line_classes_and_output(self):
        """
        Computes all the structural and texturial lines after the classification has been done
        Writes the input for a reconstruction
        :return:
        """
        lines_to_write = []
        for i, plane in enumerate(self.computed_classes):
            plane.eval_structural_lines(self.triangle_to_edge)
            if len(plane.structural_lines) < 3:
                continue
            # Write the texturial lines
            for line in plane.texturial_lines:
                # Generate a line's point of view in the direction of the normal
                tri_index = self.edge_to_triangle[line][0]
                tri_normal = self.mesh.normals[tri_index]
                fake_point_of_view = (np.array(line[:3]) + np.array(line[3:6])) / 2. + 10 * tri_normal
                lines_to_write.append(
                    "%s %s %s %s %s %s %s %s %s %s %s\n" % (line + tuple(fake_point_of_view) + (1, i)))
            # Write the structural lines
            already_written_lines = []
            for line in plane.structural_lines:
                if line in already_written_lines:
                    continue
                # Find the line's other inlier plane
                tri_indice = self.edge_to_triangle[line]
                if len(tri_indice) != 2:
                    continue
                tri_indice_other_plane = tri_indice[0] if tri_indice[1] in plane.inlier_index else tri_indice[1]
                other_plane = [i for i in range(len(self.computed_classes)) if
                               tri_indice_other_plane in self.computed_classes[i].inlier_index][0]
                plane_indice = (2, i, other_plane)
                # Generate a line's point of view in the direction of the normal
                normal_accu = self.mesh.normals[tri_indice[0]] + self.mesh.normals[tri_indice[1]]
                normal_accu /= (np.sqrt(np.dot(normal_accu, normal_accu)))
                fake_point_of_view = (np.array(line[:3]) + np.array(line[3:6])) / 2. + 10 * normal_accu
                lines_to_write.append(
                    "%s %s %s %s %s %s %s %s %s %s %s %s\n" % (line + tuple(fake_point_of_view) + plane_indice))
        with open('debug/test_lines.xyz', 'w') as output_stream:
            output_stream.write("%s\n" % len(lines_to_write))
            for line in lines_to_write:
                output_stream.write(line)

    def render_classes_occ(self):
        """
        Renders the computed classes using Open Cascade
        :return:
        """

        # Imports
        from OCC.Display.SimpleGui import init_display
        from OCC.Prs3d import Prs3d_Root_CurrentGroup, Prs3d_Presentation
        from OCC.Graphic3d import Graphic3d_ArrayOfPolygons
        from OCC.gp import gp_Pnt

        # Make colors
        class_colors = [(random.random(), random.random(), random.random()) for _ in range(len(self.computed_classes))]
        colors = [None] * len(self.mesh.vectors)
        for i, cur_class in enumerate(self.computed_classes):
            for j in cur_class.inlier_index:
                colors[j] = class_colors[i]

        # Make shapes
        array_of_polygons = Graphic3d_ArrayOfPolygons(3 * len(self.mesh.vectors),
                                                      len(self.mesh.vectors),
                                                      0,
                                                      False,
                                                      False,
                                                      True)
        for i, triangle in enumerate(self.mesh.vectors):
            if colors[i] is not None:
                for vertex in triangle:
                    pnt = gp_Pnt(*vertex.astype(np.float64))
                    array_of_polygons.AddVertex(pnt)
                array_of_polygons.AddBound(3, *colors[i])

        # Initiate the display
        display, start_display, add_menu, add_function_to_menu = init_display()
        a_presentation = Prs3d_Presentation(display._struc_mgr)
        group = Prs3d_Root_CurrentGroup(a_presentation.GetHandle()).GetObject()

        # Add the primitives
        group.AddPrimitiveArray(array_of_polygons.GetHandle())

        # Display
        a_presentation.Display()
        start_display()

    def render_classes_matplotlib(self):
        """
        Render the computed classes using Matplotlib
        :return:
        """

        # Imports
        from matplotlib import pyplot
        from mpl_toolkits import mplot3d

        # Create a new plot
        figure = pyplot.figure()
        axes = mplot3d.Axes3D(figure)

        # Make meshes out of the classes
        meshes = []
        for cur_class in self.computed_classes:
            if len(cur_class.inlier_index) > 2 or True:
                data = {}
                data['vectors'] = self.mesh.vertices[self.mesh.faces[cur_class.inlier_index]]
                data['normals'] = self.mesh.normals[cur_class.inlier_index]
                meshes.append(data.copy())

        # Render all the meshes with random colors
        for m in meshes:
            collection = mplot3d.art3d.Poly3DCollection(m['vectors'])
            face_color = np.random.uniform(0, 1, 3)
            collection.set_facecolor(face_color)
            axes.add_collection3d(collection)

        # Auto scale to the mesh size
        scale = np.stack((m['vectors'].reshape((3, -1)).min(1), m['vectors'].reshape((3, -1)).max(1)), axis=1)
        axes.auto_scale_xyz(scale, scale, scale)

        # Show the plot to the screen
        pyplot.show()

    def save_planes_as_json(self, out_path):
        sorted_classes = sorted(self.computed_classes, key=lambda x: -x.triangle_area_accumulator)
        json_planes = {"planes":
                           [{"inlier": x.average_point.tolist(), "normal": x.normal.tolist()} for x in sorted_classes],
                       "map": {"NOMAP": 0},
                       "labels": [True],
                       'bbox': self.mesh.vertices.min(0).tolist() + self.mesh.vertices.max(0).tolist(),
                       "nbPlanes": 180}
        with open(out_path, "w") as out_file:
            json.dump(json_planes, out_file)


def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", help="stl file whose class have to be computed", required=True)
    parser.add_argument("--epsilon_point", help="Threshold on point for a triangle to belong to a class",
                        default=0.01)
    parser.add_argument("--epsilon_normal", help="Threshold on normal for a triangle to belong to a class",
                        default=0.005)
    parser.add_argument("--sigma_point", help="Threshold on point for two classes to be matched",
                        default=0.04)
    parser.add_argument("--sigma_normal", help="Threshold on normal for two classes to be matched",
                        default=0.001)
    backend_parser = parser.add_mutually_exclusive_group(required=True)
    backend_parser.add_argument('--matplotlib', dest='backend_mpl', action='store_true', help="Matplotlib backend")
    backend_parser.add_argument('--occ', dest='backend_mpl', action='store_false', help="Open Cascade backend")
    args = parser.parse_args()

    # Compute the class
    class_computer = ClassComputer(args.data, args.epsilon_point, args.epsilon_normal,
                                   args.sigma_point, args.sigma_normal)
    class_computer.compute_classes_from_file_by_growing()

    # Output obtained planes
    class_computer.save_planes_as_json(join("debug", "planes.json"))

    # Compute the structural lines
    class_computer.compute_line_classes_and_output()

    # Display it
    if args.backend_mpl:
        class_computer.render_classes_matplotlib()
    else:
        class_computer.render_classes_occ()


if __name__ == "__main__":
    main()
