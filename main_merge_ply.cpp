// STD
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

// External
#include "tinyply/tinyply.h"

using namespace std;
using namespace tinyply;

struct float3 { float x, y, z; };
struct uint3 { uint32_t x, y, z; };
struct uint4 { uint8_t x, y, z, w; };

struct memory_buffer : public std::streambuf
{
    char * p_start {nullptr};
    char * p_end {nullptr};
    size_t size;

    memory_buffer(char const * first_elem, size_t size)
        : p_start(const_cast<char*>(first_elem)), p_end(p_start + size), size(size)
    {
        setg(p_start, p_start, p_end);
    }

    pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
    {
        if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
        else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
        return gptr() - p_start;
    }

    pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
    {
        return seekoff(pos, std::ios_base::beg, which);
    }
};

struct memory_stream : virtual memory_buffer, public std::istream
{
    memory_stream(char const * first_elem, size_t size)
        : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf*>(this)) {}
};

inline vector<uint8_t> read_file_binary(const string & pathToFile)
{
    ifstream file(pathToFile, ios::binary);
    vector<uint8_t> fileBufferBytes;

    if (file.is_open())
    {
        file.seekg(0, ios::end);
        size_t sizeBytes = file.tellg();
        file.seekg(0, ios::beg);
        fileBufferBytes.resize(sizeBytes);
        if (file.read((char*)fileBufferBytes.data(), sizeBytes)) return fileBufferBytes;
    }
    else throw runtime_error("could not open binary ifstream to path " + pathToFile);
    return fileBufferBytes;
}

void loadMesh(const string& path, vector<float3> &_points, vector<uint3> &_faces, vector<uint4> &_colors)
{

    cout << "Now Reading: " << path << endl;

    unique_ptr<istream> file_stream;
    vector<uint8_t> byte_buffer;

    try
    {
        // Preload into memory
        byte_buffer = read_file_binary(path);
        file_stream.reset(new memory_stream((char*)byte_buffer.data(), byte_buffer.size()));

        // Check if the stream was correctly open
        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + path);

        // Compute file size
        file_stream->seekg(0, std::ios::end);
        const float size_mb = file_stream->tellg() * float(1e-6);
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
        catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }
        try { colors = file.request_properties_from_element("face", { "red", "green", "blue", "alpha" }); }
        catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }

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
    for(int j=0; j < faces.size(); j++)
      totalFaces.push_back({faces[j].x + faceCursor, 
                            faces[j].y + faceCursor, 
                            faces[j].z + faceCursor});
    faceCursor += points.size();

    // Merge the colors
    totalFaceColors.reserve(totalFaceColors.size() + colors.size());
    totalFaceColors.insert(totalFaceColors.end(), colors.begin(), colors.end());
  }
  // Save the final mesh
  saveMesh(argv[1], totalPoints, totalFaces, totalFaceColors);
  return 0;
}