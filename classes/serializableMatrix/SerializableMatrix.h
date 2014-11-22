/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SerializableMatrix_H
#define SerializableMatrix_H

#include <stdio.h>

#include <string>
#include <vector>


template <typename T>
class SerializableMatrix
{
public:

	bool load(std::string path)
	{
		FILE *fd = fopen(path.c_str(), "r");
		if (fd != NULL) {
			unsigned int rows, columns, depth;
			fread(&rows, 1, sizeof(unsigned int), fd);
			fread(&columns, 1, sizeof(unsigned int), fd);
			fread(&depth, 1, sizeof(unsigned int), fd);
			resize(rows, columns, depth);
			fread(&data[0], rows*columns*depth, sizeof(T), fd);
			fclose(fd);
			return true;
		}
		else
		{
			printf ("Can't open %s for reading\n", path.c_str());
			fflush(stdout);
			exit(-3);
			return false;
		}
	}

	void save(std::string path)
	{
		FILE *fd = fopen(path.c_str(), "w");
		if (fd != NULL)
		{
			fwrite(&height,  1,                  sizeof(unsigned int), fd);
			fwrite(&width,   1,                  sizeof(unsigned int), fd);
			fwrite(&depth,   1,                  sizeof(unsigned int), fd);
			fwrite(&data[0], height*width*depth, sizeof(T),            fd);
			fclose(fd);
		}
		else
		{
			printf ("Can't open %s for writting\n", path.c_str());
			fflush(stdout);
			exit(-1);
		}
	}

	SerializableMatrix()
	{
		width = height = depth = 0;
		data.resize(1);
	}

	void resize(int newWidth, int newHeight=1, int newDepth=1)
	{
		width = newWidth;
		height = newHeight;
		depth = newDepth;
		data.resize(width*height*depth);
	}

	inline void memset0()
	{
		memset(getDataPointer(), 0, sizeof(T)*width*height*depth);
	}

	inline std::vector<T> *getVector()
	{
		return &data;
	}

	inline unsigned int getWidth()
	{
		return width;
	}

	inline unsigned int getHeight()
	{
		return height;
	}

	inline unsigned int getDepth()
	{
		return depth;
	}

	inline unsigned int getSize()
	{
		if (width*height*depth != data.size()) exit(-934);
		return width*height*depth;
	}

	inline T *getDataPointer()
	{
		if (data.size() > 0)
			return &data[0];
		return NULL;
	}

	inline T get1D(unsigned int x)
	{
		if (data.size() > x) return data[x];
		else throw std::string("SerializableMatrix::get1D() out of bounds.");
	}

	inline T get2D(unsigned int x, unsigned int y)
	{
		if (data.size() > x + y*width) return data[x + y*width];
		else throw std::string("SerializableMatrix::get2D() out of bounds.");
	}

	inline T get3D(unsigned int x, unsigned int y, unsigned int z)
	{
		if (data.size() > x + y*width + z*height*width) return data[x + y*width + z*height*width];
		else throw std::string("SerializableMatrix::get3D() out of bounds.");
	}

	inline void set1D(unsigned int x, T val)
	{
		if (data.size() > x) data[x] = val;
		else throw std::string("SerializableMatrix::set1D() out of bounds.");
	}

	inline void set2D(unsigned int x, unsigned int y, T val)
	{
		if (data.size() > x + y*width) data[x + y*width] = val;
		else throw std::string("SerializableMatrix::set2D() out of bounds.");
	}

	inline void set3D(unsigned int x, unsigned int y, unsigned int z, T val)
	{
		if (data.size() > x + y*width + z*height*width) data[x + y*width + z*height*width] = val;
		else throw std::string("SerializableMatrix::set2D() out of bounds.");
	}

	inline void inc1D(unsigned int x)
	{
		if (data.size() > x) data[x] = data[x] + 1;
		else throw std::string("SerializableMatrix::inc1D() out of bounds.");
	}

	inline void inc2D(unsigned int x, unsigned int y)
	{
		if (data.size() > x + y*width) data[x + y*width] = data[x + y*width] + 1;
		else throw std::string("SerializableMatrix::inc2D() out of bounds.");
	}

	inline void inc3D(unsigned int x, unsigned int y, unsigned int z)
	{
		if (data.size() > x + y*width + z*height*width) data[x + y*width + z*height*width] = data[x + y*width + z*height*width] + 1;
		else throw std::string("SerializableMatrix::inc3D() out of bounds.");
	}

// 	inline void blur()
// 	{
// 		for (int xp=1;xp<width-1;xp++)
// 		{
// 			for (int yp=1;yp<height-1;yp++)
// 			{
// 				for (int zp=1;zp<depth-1;zp++)
// 				{
// 					set3D(xp, yp, zp, 
// 						(
// 						 get3D(xp-1,yp-1,zp-1) + get3D(xp-1,yp,zp-1) + get3D(xp-1,yp+1,zp-1) +
// 						 get3D(xp-1,yp-1,zp)   + get3D(xp-1,yp,zp)   + get3D(xp-1,yp+1,zp)   +
// 						 get3D(xp-1,yp-1,zp+1) + get3D(xp-1,yp,zp+1) + get3D(xp-1,yp+1,zp+1) +
// 
// 						 get3D(xp,yp-1,zp-1) + get3D(xp,yp,zp-1) + get3D(xp,yp+1,zp-1) +
// 						 get3D(xp,yp-1,zp)   + get3D(xp,yp,zp)   + get3D(xp,yp+1,zp)   +
// 						 get3D(xp,yp-1,zp+1) + get3D(xp,yp,zp+1) + get3D(xp,yp+1,zp+1) +
// 
// 						 get3D(xp+1,yp-1,zp-1) + get3D(xp+1,yp,zp-1) + get3D(xp+1,yp+1,zp-1) +
// 						 get3D(xp+1,yp-1,zp)   + get3D(xp+1,yp,zp)   + get3D(xp+1,yp+1,zp)   +
// 						 get3D(xp+1,yp-1,zp+1) + get3D(xp+1,yp,zp+1) + get3D(xp+1,yp+1,zp+1)
// 						)/3
// 						);
// 				}
// 			}
// 		}
// 	}

private:
	unsigned int width;
	unsigned int height;
	unsigned int depth;
	std::vector<T> data;
};

#endif
