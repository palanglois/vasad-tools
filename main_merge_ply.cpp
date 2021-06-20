// STD
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cstring>

// External
#include "tinyply/tinyply.h"

// Own
#include "plyInline.h"

using namespace std;
using namespace tinyply;

void loadMesh(const string& path, vector<float3> &_points, vector<uint3> &_faces, vector<uint4> &_colors)
{

    cout << "Now Reading: " << path << endl;

    unique_ptr<istream> file_stream;
    vector<uint8_t> byte_buffer;
    bool threeChannels = false;

    try
    {
        // Preload into memory
        byte_buffer = read_file_binary(path);
        file_stream = make_unique<memory_stream>((char*)byte_buffer.data(), byte_buffer.size());

        // Check if the stream was correctly open
        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + path);

        // Set the cursor at the beginning of the file
        file_stream->seekg(0, std::ios::beg);

        // Parse the PLY file's header
        PlyFile file;
        file.parse_header(*file_stream);

        // Prepare the data containers
        shared_ptr<PlyData> vertices, faces, colors;
   
        // Requesting the relevant properties
        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }
        try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        catch (const exception & e) {
            try { faces = file.request_properties_from_element("face", { "vertex_index" }, 3); }
            catch (const exception & e) {cerr << "tinyply exception: " << e.what() << endl;}
        }
        try { colors = file.request_properties_from_element("face", { "red", "green", "blue", "alpha" }); }
        catch (const exception & e) {
            try { colors = file.request_properties_from_element("face", { "red", "green", "blue"}); threeChannels=true;}
            catch (const exception & e) {cerr << "tinyply exception: " << e.what() << endl;}
        }

        // Actual parsing
        file.read(*file_stream);

        // Process the vertices
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        _points = vector<float3>(vertices->count);
        memcpy(_points.data(), vertices->buffer.get(), numVerticesBytes);

        // Processing the faces
        const size_t numFacesBytes = faces->buffer.size_bytes();
        _faces = vector<uint3>(faces->count);
        memcpy(_faces.data(), faces->buffer.get(), numFacesBytes);

        // Processing the colors
        const size_t numColorsBytes = colors->buffer.size_bytes();
        _colors = vector<uint4>(colors->count);
        if(threeChannels)
        {
            vector<uint3c> rawColors(colors->count);
            memcpy(rawColors.data(), colors->buffer.get(), numColorsBytes);
            for(int i=0; i < rawColors.size(); i++)
            {
                _colors[i].x = rawColors[i].x;
                _colors[i].y = rawColors[i].y;
                _colors[i].z = rawColors[i].z;
                _colors[i].w = 255;
            }
        }
        else
            memcpy(_colors.data(), colors->buffer.get(), numColorsBytes);
    }
    catch (const exception & e)
    {
        cerr << "Caught tinyply exception: " << e.what() << endl;
    }
}

void saveMesh(const string& path, vector<float3> &_points, vector<uint3> &_faces, vector<uint4> &_colors)
{
    // Open the output stream in binary mode
    filebuf fb_binary;
    fb_binary.open(path, ios::out | ios::binary);
    ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw runtime_error("failed to open " + path);

    // Creating the properties of the output file
    PlyFile out_file;

    // Points
    out_file.add_properties_to_element("vertex", { "x", "y", "z" }, 
        Type::FLOAT32, _points.size(), reinterpret_cast<uint8_t*>(_points.data()), Type::INVALID, 0);

    // Faces
    out_file.add_properties_to_element("face", { "vertex_indices" },
        Type::UINT32, _faces.size(), reinterpret_cast<uint8_t*>(_faces.data()), Type::UINT8, 3);

    // Face colors
    out_file.add_properties_to_element("face", { "red", "green", "blue", "alpha" },
        Type::UINT8, _colors.size(), reinterpret_cast<uint8_t*>(_colors.data()), Type::INVALID, 0);

    // Write the output binary file
    out_file.write(outstream_binary, true);
}

int main(int argc, char *argv[])
{
  // Display help
  if(argc < 4)
  {
    cout << "Usage: './mergePly outFile.ply fileToMerge1.ply fileToMerge2.ply ...'" << endl;
    return 0;
  }

  // Containers for the final shape
  vector<float3> totalPoints; 
  vector<uint3> totalFaces; 
  vector<uint4> totalFaceColors;
  int faceCursor = 0;
  for(int i=2; i < argc;i++)
  {
    // Load the current shape
    vector<float3> points; 
    vector<uint3> faces; 
    vector<uint4> colors;
    loadMesh(argv[i], points, faces, colors);

    // Merge the points
    totalPoints.reserve(totalPoints.size() + points.size());
    totalPoints.insert(totalPoints.end(), points.begin(), points.end());

    // Merge the faces
    totalFaces.reserve(totalFaces.size() + faces.size());
    for(auto & face : faces)
      totalFaces.push_back({face.x + faceCursor,
                            face.y + faceCursor,
                            face.z + faceCursor});
    faceCursor += points.size();

    // Merge the colors
    totalFaceColors.reserve(totalFaceColors.size() + colors.size());
    totalFaceColors.insert(totalFaceColors.end(), colors.begin(), colors.end());
  }
  // Save the final mesh
  saveMesh(argv[1], totalPoints, totalFaces, totalFaceColors);
  return 0;
}
