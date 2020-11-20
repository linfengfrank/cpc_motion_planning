/************************************************************
 * Implememnt file for map loading and providing
 * This file implement functions of loading map from a bmp file and provide interface
 * for users to get map data
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "glb_plan/def.h"
#include "glb_plan/map.h"

CMap::CMap()
{
    m_data = NULL;
    m_width = m_height = 0;
}

BOOL CMap::Load(const char *filename) {
	double sinc, cosc;
    int patch;
    const int ALIGN = 4;

    FILE *file = fopen(filename, "rb");
    if (file == NULL) return FALSE;

    int nread;

    /* read file headers */
    BMP_HEADER header;
    nread = fread(&header, 1, sizeof(header), file);
    if (nread != sizeof(header)) { printf("Read bmp header error!\n"); goto RETURN; }

    printf("BMP header read, offset %d\n", header.offset);

    BMP_INFOHEADER infoheader;
    nread = fread(&infoheader, 1, sizeof(infoheader), file);
    if (nread != sizeof(infoheader)) { printf("Read info header error!\n"); goto RETURN; }

    printf("BMP info header read, width %d, height %d, image size %d\n", infoheader.width, infoheader.height, infoheader.imagesize);

    /* read image data */
    m_width = infoheader.width;
    m_height = infoheader.height;

    patch = ALIGN-m_width%ALIGN;

    m_data = (BYTE *)malloc(m_width*m_height);

    fseek(file, header.offset, SEEK_SET);

    BYTE data[32];
    for (int i=0; i<m_height; i++) {
        for (int j=0; j<m_width; j++) {
            int ip = m_height-1-i;              //reverse order of rows
            fread(m_data+ip*m_width+j, 1, 1, file);
        }
        fseek(file, patch, SEEK_CUR);           //skip patched bytes for alignment
    }

    //Init coordination
    m_x0 = 1088;
    m_y0 = 977;
    m_scale = 0.0328;            //distance per pixel
    m_theta = 4.45*M_PI/180;

	sinc = sin(m_theta);
	cosc = cos(m_theta);

	m_transform[0][0] = cosc;
	m_transform[0][1] = -sinc;
	m_transform[1][0] = sinc;
	m_transform[1][1] = cosc;

RETURN:
    fclose(file);
    return TRUE;
}

BYTE CMap::Grey(int x, int y) {
    int i = x;          //rows for x
    int j = y;          //cols fy y

    if (i < 0 || i >= m_height || j < 0 || j >= m_width) return 0xff;

    BYTE pixel = m_data[i*m_width+j];

    return pixel;
}

BYTE CMap::Grey(double x, double y) {
    POSITION pos;
    POINT pt;
    pos.x = x; pos.y = y;

    Pos2Pt(&pos, &pt);

    return Grey(pt.x, pt.y);
}

void CMap::Pt2Pos(POINT *pt, POSITION *pos) {
	//position from origin
	int x_pt = pt->x - m_x0;
	int y_pt = pt->y - m_y0;

	//transform of rotation
	double x = x_pt*m_transform[0][0] + y_pt*m_transform[0][1];
	double y = x_pt*m_transform[1][0] + y_pt*m_transform[1][1];

	//scale
	x *= m_scale;
	y *= m_scale;

	pos->x = x;
	pos->y = y;
}

void CMap::Pos2Pt(POSITION *pos, POINT *pt) {
	//scale
	double x = pos->x/m_scale;
	double y = pos->y/m_scale;

	//rotation back
	double x_rot = x*m_transform[0][0] + y*m_transform[1][0];
	double y_rot = x*m_transform[0][1] + y*m_transform[1][1];

	//shift
	pt->x = (int)(x_rot+0.5) + m_x0;
	pt->y = (int)(y_rot+0.5) + m_y0;
}

