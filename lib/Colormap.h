#ifndef BIM_DATA_COLORMAP_H
#define BIM_DATA_COLORMAP_H

#include <utility>

class ColorBase {
public:
    ColorBase() {
        rc = 127;
        gc = 127;
        bc = 127;
    }

    ColorBase(unsigned char _r, unsigned char _g, unsigned char _b) : rc(_r), gc(_g), bc(_b) {}

    ColorBase &operator=(const ColorBase &left) {
        rc = left.r();
        gc = left.g();
        bc = left.b();
        return *this;
    }

    unsigned char r() const { return rc; }

    unsigned char g() const { return gc; }

    unsigned char b() const { return bc; }

private:
    unsigned char rc;
    unsigned char gc;
    unsigned char bc;
};

class Colormap {
public:
    typedef ColorBase Color;

    explicit Colormap(std::map<int, Color> _map) : map(std::move(_map)) {}

    const Color &operator[](int i) const { return map.at(i); }

private:
    const std::map<int, Color> map;
};


#endif //BIM_DATA_COLORMAP_H
