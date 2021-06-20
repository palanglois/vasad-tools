#ifndef BIM_DATA_PLYINLINE_H
#define BIM_DATA_PLYINLINE_H

using namespace std;

struct float3 { float x, y, z; };
struct uint3 { uint32_t x, y, z; };
struct uint3c { uint8_t x, y, z; };
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

#endif //BIM_DATA_PLYINLINE_H
