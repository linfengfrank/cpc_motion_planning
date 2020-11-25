#ifndef CMAP_H
#define CMAP_H

#include "glb_plan/def.h"
#include "cuda_geometry/cuda_geometry.cuh"

#pragma pack(1)
struct BMP_HEADER {
    uint16_t    type;                   //Magic identifier   文件格式
    uint32_t    size;                   //File size in bytes 文件大小
    uint16_t    reserved1, reserved2;   //保留区 默认设置为0
    uint32_t    offset;                 //Offset to image data, bytes文件头到图像数据信息便宜字节
};

struct BMP_INFOHEADER {
    uint32_t    size;                   //Header size in bytes infoheader
    int32_t     width,height;           //Width and height of image
    uint16_t    planes;                 //Number of colour planes
    uint16_t    bits;                   //Bits per pixel, must be 1, 4, 8(256 color) or 24(true color)
    uint32_t    compression;            //Compression type
    uint32_t    imagesize;              //Image size in bytes
    int32_t     xresolution,yresolution;    //Pixels per meter
    int32_t     ncolours;               //Number of colours
    uint32_t    importantcolours;       //Important colours
};

struct BMP_PALETTE {
    BYTE    rgbBlue;        //蓝色的亮度(值范围为0-255)
    BYTE    rgbGreen;       //绿色的亮度(值范围为0-255)
    BYTE    rgbRed;         //红色的亮度(值范围为0-255)
	BYTE    reserved;	    //保留，必须为0
};
#pragma pack()

struct POINT {
	int16_t	x;
	int16_t y;
};

struct POSITION {
	double x;
	double y;
};

struct VEC2D {
    double x;
    double y;
};

struct CROP_AREA {
    double x0, y0;
    double rx, ry;      //range of x, range of y
    double dx;          //dx=dy
};

struct CROP_MAP {
    BYTE *data;
    int nx, ny;             //numbers of elements along x and y axis
};

class CMap
{
public:
    CMap();

protected:
    //image data
    BYTE *m_data;
    int m_width, m_height;

    //coordination
    int m_x0, m_y0;
    double m_scale;
    double m_theta;
    double m_transform[2][2];


public:
    BOOL Load(const char *filename);

    int Width() { return m_width; }
    int Height() { return m_height; }

    BYTE Grey(int x, int y);

    BYTE Grey(double x, double y);

	void Pos2Pt(POSITION *pos, POINT *pt);		//map: pos->pt
	void Pt2Pos(POINT *pt, POSITION *pos);		//map: pt->pos
  void Crop(CROP_AREA &crop, CROP_MAP &map);
  void Crop(CUDA_GEO::pos origin, int x_size, int y_size, float grid_step, unsigned char* occu_data);
};

#endif // CMAP_H
