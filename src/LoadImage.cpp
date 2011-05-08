#include "LoadImage.h"
#include "algebra3.h"
#include <string>

using namespace std;

// load a bitmap with freeimage
bool loadBitmap(string filename, FIBITMAP* &bitmap) {
    // get the file format
    FREE_IMAGE_FORMAT format = FreeImage_GetFileType(filename.c_str(), 0);
    if (format == FIF_UNKNOWN)
        format = FreeImage_GetFIFFromFilename(filename.c_str());
    if (format == FIF_UNKNOWN)
        return false;

    // load the image
    bitmap = FreeImage_Load(format, filename.c_str());
    if (!bitmap)
        return false;

    return true;
}

// load a texture into opengl with freeimage
bool loadTexture(string filename, GLuint &texture) {
    FIBITMAP *bitmap = NULL;
    if (!loadBitmap(filename, bitmap))
        return false;

    // convert to 32 bit bit-depth
    FIBITMAP *bitmap32 = FreeImage_ConvertTo32Bits(bitmap);
    FreeImage_Unload(bitmap);
    if (!bitmap32)
        return false;
    bitmap = bitmap32;

    // get bits and dimensions
    BYTE *bits = FreeImage_GetBits(bitmap);
    int w = FreeImage_GetWidth(bitmap);
    int h = FreeImage_GetHeight(bitmap);

    // get bit order
    int order = GL_BGRA;

    // upload texture to opengl
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 4, w, h, order, GL_UNSIGNED_BYTE, (GLvoid*)bits);

    // forget our copy of the bitmap now that it's stored the card
    FreeImage_Unload(bitmap);

    return true;
}

// load a height map and normal map (computed from the height map) into opengl with freeimage
bool loadHeightAndNormalMaps(string filename, GLuint &heightmap, GLuint &normalmap, double zScale) {
    FIBITMAP *bitmap = NULL;
    if (!loadBitmap(filename, bitmap))
        return false;

    // convert to 8-bit greyscale
    FIBITMAP *bitmapGrey = FreeImage_ConvertToGreyscale(bitmap);
    FreeImage_Unload(bitmap);
    if (!bitmapGrey)
        return false;
    bitmap = bitmapGrey;

    // get bits and dimensions
    BYTE *bits = FreeImage_GetBits(bitmap);
    int w = FreeImage_GetWidth(bitmap);
    int h = FreeImage_GetHeight(bitmap);

    // allocate a normal map
    FIBITMAP *normals = FreeImage_Allocate(w, h, 24);
    if (!normals) { // failed to alloc
        FreeImage_Unload(bitmap); // do or do not there is no try
        return false;
    }

    // upload heightmap to opengl
    glGenTextures(1, &heightmap);
    glBindTexture(GL_TEXTURE_2D, heightmap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_LUMINANCE, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, (GLvoid*)bits);

    // compute normals
    computeNormalMapFromHeightMap(bitmap, normals, zScale);

    // get bit order
    int order = GL_BGR;
    #if FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_RGB
    order = GL_RGB;
    #endif

    bits = FreeImage_GetBits(normals);

    // upload heightmap to opengl
    glGenTextures(1, &normalmap);
    glBindTexture(GL_TEXTURE_2D, normalmap);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, w, h, 0, order, GL_UNSIGNED_BYTE, (GLvoid*)bits);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // forget our copy of the bitmap now that it's stored the card
    FreeImage_Unload(bitmap);
    FreeImage_Unload(normals);

    return true;
}

namespace {
    // helper function for getting pixels from a bitmap
    inline double pix(FIBITMAP* map, int x, int y) {
        unsigned char val;
        FreeImage_GetPixelIndex(map, x, y, &val); // slow because of bounds checks, but who cares?  this is a preprocess.
        return val / 255.0;
    }
}

// called by the height & normal map loader, computes normals from height map
void computeNormalMapFromHeightMap(FIBITMAP *heights, FIBITMAP *normals, double zScale) {
    RGBQUAD color;
    int w = FreeImage_GetWidth(heights);
    int h = FreeImage_GetHeight(heights);

    // march xPrev,x,xNext triple over range, assuming the texture wraps
    for (int xPrev = w-2, x = w-1, xNext = 0; xNext < w; xPrev=x, x=xNext++) {

        // similarly, march yPrev,y,yNext triple
        for (int yPrev = h-2, y = h-1, yNext = 0; yNext < h; yPrev=y, y=yNext++) {
            // @TODO: Compute the normal from the height map
            // HINT: The pix function may help.
			double center = pix(heights,x,y);
			double xHeight = pix(heights,xPrev,y);
			double yHeight = pix(heights,x,yPrev);
			vec3 xVec = vec3(xPrev,y,xHeight);
			vec3 yVec = vec3(x,yPrev,yHeight);
			vec3 centVec = vec3(x,y,center);
			xVec = xVec-centVec;
			yVec = yVec-centVec;
            vec3 normal = (xVec^yVec).normalize();
			for(int i = 0; i<3; i++){
				normal[i]=(normal[i]+1)*127.5;
			}

            color.rgbRed = BYTE(normal[0]);
            color.rgbGreen = BYTE(normal[1]);
            color.rgbBlue = BYTE(normal[2]);
            FreeImage_SetPixelColor(normals,x,y,&color);
        }
    }
}

