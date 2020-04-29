import bpy
import json
import argparse
from os.path import join, abspath, dirname
from os import makedirs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=str, default="data")
    args = parser.parse_args()
    file_name = bpy.data.filepath.split(sep='/')[-1].split(sep='.')[0]
    args.output = join(dirname(dirname(__file__)), args.output, file_name)
    makedirs(args.output, exist_ok=True)
    json_obj = {"cam_loc": [], "filename": file_name}
    for i in range(1, bpy.context.scene.frame_end + 1):
        bpy.context.scene.frame_set(i)
        loc = bpy.context.scene.camera.location
        json_obj["cam_loc"].append([x for x in loc])
    with open(join(args.output, "cam_loc.json"), 'w') as out_file:
        json.dump(json_obj, out_file)
    bpy.ops.export_scene.obj(filepath=join(args.output, "model.obj"), check_existing=True, filter_glob="*.obj;*.mtl", 
        use_selection=False, use_animation=False, use_mesh_modifiers=True, use_edges=True, use_smooth_groups=False, 
        use_smooth_groups_bitflags=False, use_normals=True, use_uvs=True, use_materials=False, use_triangles=True, 
        use_nurbs=False, use_vertex_groups=False, use_blen_objects=True, group_by_object=False, group_by_material=False, 
        keep_vertex_order=False, global_scale=1, path_mode='AUTO', axis_forward='Y', axis_up='Z')
        

if __name__ == "__main__":
    main()
