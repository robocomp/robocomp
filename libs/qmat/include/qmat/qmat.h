/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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

#ifndef QMAT_H
#define QMAT_H

#include <QtCore>
// #include <math.h>
#include <vector>
#include <limits>

//#include <eigen3/Eigen/Core>

#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>


#ifdef PYTHON_BINDINGS_SUPPORT
#include <boost/python.hpp>
#include <boost/python/list.hpp>
static std::vector<float> to_std_vector_QVec(const boost::python::list &ns)
{
	std::vector<float> ret;
	for (int i=0; i<len(ns); ++i)
	{
		ret.push_back(boost::python::extract<float>(ns[i]));
	}
	return ret;
}

static std::vector<int> to_std_vector_Index(const boost::python::tuple &ns)
{
	std::vector<int32_t> ret;
	for (int i=0; i<len(ns); ++i)
	{
		ret.push_back(boost::python::extract<int32_t>(ns[i]));
	}
	return ret;
}
#endif

#define MAX_DIMENSION 100


namespace RMat
{
	typedef float T;
	class QVec;
	class DataBuffer: public QSharedData
	{
	public:
		DataBuffer(const int size): QSharedData(), dataSize(size), data(new T[dataSize]) { }
		DataBuffer(const DataBuffer &tensorData): QSharedData(), dataSize(tensorData.dataSize), data(new T[dataSize]) { memcpy(getWriteData() , tensorData.getReadData(), dataSize*sizeof(T)); }
		~DataBuffer()                       { delete[] data; }
		inline const T *getReadData() const { return data; }
		inline T *getWriteData()            { return data; }
		int size()                          { return dataSize;}

		int dataSize;
		T *data;
	};

	class QMat
	{
	protected:
		int cols;
		int rows;
		QSharedDataPointer< DataBuffer > data;
	public:
		QMat();
		QMat( int rows, int columns );
		QMat( int rows, int columns, const T &value );
		QMat( int rows );                                                 // Deprecated
		QMat( int rows, const T &value );                                 // Deprecated
		QMat( const QMat &matrix );
		QMat( const QList< QVec > & vectorList);
		QMat( const QVec & vector, const bool rowVector = false);
		QMat( const gsl_matrix *matrix);

		~QMat() {};



		QMat copy();
		QMat & inject ( const QMat & matrix, const int foff, const int coff );
		//Access functions
		inline T & operator() (const int row, const int column)          { return getWriteData()[row*cols+column]; }
		inline T operator() (const int row, const int column) const      { return getReadData()[row*cols+column]; }
		inline T & operator() (const int row)                            { return getWriteData()[row*cols]; }
		inline T operator() (const int row) const                        { return getReadData()[row*cols]; }
		inline void setItem(int i1, int i2, const T val)                 { operator()(i1, i2) = val; }
		inline T getItem(int i1, int i2) const                           { return operator()(i1, i2); }
#ifdef PYTHON_BINDINGS_SUPPORT
		inline void setItemV(boost::python::tuple &v, T val)
		{
			std::vector<int32_t> i = to_std_vector_Index(v);
			operator()(i[0], i[1]) = val;
		}
		inline T getItemV(boost::python::tuple &v) const
		{
			std::vector<int32_t> i = to_std_vector_Index(v);
			return operator()(i[0], i[1]);
		}
#endif
		inline int nRows() const                                         { return rows; }
		inline int nCols() const                                         { return cols; }
		int getDataSize() const                                          { return rows*cols; }
		inline const T *getReadData() const
		{
			return data->getReadData();
		}
		inline T *getWriteData()
		{
			return data->getWriteData();
		}
		T* toData()                                                      { return getWriteData(); } //deprecated use getWriteData
		const T* toDataConst() const                                     { return getReadData();} //deprecated use getReadData
		int getCols() const                                              { return cols; }
		int getRows() const                                              { return rows; }

		// Matrix - Matrix operators
		QMat & operator= ( const QMat & A );
		QMat   operator* ( const QMat & A ) const;
		QMat   operator+ ( const QMat & A ) const;
		QMat   operator- ( const QMat & A ) const;
		QMat   operator& ( const QMat & A ) const;                              // dot multiply
		QMat   operator/ ( const QMat & A ) const;
		QMat & operator&= ( const QMat & A );                                  // dot multiply
		QMat & operator+= ( const QMat & A );
		QMat & operator-= ( const QMat & A );
		QMat & operator/= ( const QMat & A );
		QMat operator^ (const QMat & A);                                       // cross product
		bool operator== (const QMat & A);

		// Matrix-vector operators
		QVec operator*(const QVec &vector) const;

		//Matrix - scalar operators
		QMat & operator= ( T a );
		QMat  operator* ( const T f ) const;
		QMat  operator+ ( const T f ) const;
		QMat  operator- ( const T f ) const;
		QMat  operator& ( const T f ) const;
		QMat  operator/ ( const T f ) const;
		QMat & operator&= ( const T &f );
		QMat & operator+= ( const T &f );
		QMat & operator-= ( const T &f );
		QMat & operator/= ( const T &f );

		//Matrix - scalar operations

		void set ( T v );
		void makeDiagonal ( T d );        // Deprecated
		void diagonal ( T d )             { makeDiagonal ( d ); };  //Renaming

		//Matrix - matrix operations

		QMat & makeUnitary();
		QVec getDiagonal( );
		QMat transpose() const;
		QMat t()                   { return transpose(); }; //deprecated
		T determinant( ) const;
		T trace( );
		QMat  invert( ) const;
//		QMat  inverse( );        //deprecated use invert
//		QMat  inverse( ) const;  //deprecated use invert
		void ones( );
		QMat & makeUnitModulus( );
		QMat & makeIdentity();
		bool isSquare ( const QMat &A ) const;
		bool is3ColumnVector( const QMat &A ) const;
		inline bool is3ColumnVector( ) const;
		inline bool canAllocateRotationMatrix() const;
		inline bool isSquare() const;
		inline bool isColVector() const         { return ( cols==1 and rows > 0 );};
		bool isEmpty()                          { return getDataSize() == 0;};
		int minDim ( const QMat &A );
		int minDim();
		int maxDim ( const QMat &A );
		int maxDim();
		inline bool equalSize ( const QMat & A, const QMat & B ) const;
		void print ( const QString & s ) const;
		void prints ( const std::string & s ) const { print(QString::fromStdString(s)); }

		QMat sqrt();                          // Element square root
		QMat cholesky();
		QMat eigenValsVectors ( QVec & vals );
		void SVD(QMat & U, QMat & D, QMat & V);
		QMat makeDefPos();
		QMat matSqrt();                                                        //!< Matrix square root
		T vectorNormL2() const;                       // deprecated to vec
		QMat fromStdVector( const std::vector<T> &);  // deprecated to vec
		QMat toCrossProdForm() const;                 // deprecated to vec
		QVec toVector() const;
		QVec extractAnglesR() const;
		QVec extractAnglesR_min() const;
		bool extractAnglesR2(QVec &a, QVec &b) const;


		//Access operations
		void setCol(const int row, QVec vector);
		const QVec getCol(const int col) const;
		void setRow(const int row, QVec vector);
		void setRow(const int row, QVector< T > vector);
		const QVec getRow(const int row) const;
		const QMat getSubmatrix(const int firstRow, const int lastRow, const int firstCol, const int lastCol) const;
		QMat getFil ( int n );                      //deprecated to getRow

		//Static members
		static QMat afinTransformFromIntervals( const QList<QPair<QPointF,QPointF> > & intervals);
		static QMat diagonal ( const QMat &v );                                //!<Static version of void diagonal() to be used in initializing variables
		static QMat gaussian ( const int fi, const float mean, const float stdev); //!< Normal distributed column vector// deprecated to vec
		static QMat identity ( const int m );                                  //!<Static version of void loadIdentity to be used in initializing variables.
		static QMat makeDiagonal ( const QVec &v );                            // Create a diagonal matrix from a vector
		static QMat ones ( const int m, const int n );                         //!<Static version of void loadIdentity to be used in initializing variables.
		static QMat random ( const int fi, const int co );                     //!<Static version of void random() to be used in initializing variables
		static QMat vec3(T x, T y, T z);                                       // deprecated to vec
		static QMat zeroes ( const int m, const int n );                       // Deprecated. Use zeros
		static QMat zeros ( const int m, const int n );                        //!<Static version for creating a zeroed matrix.


		public:
		friend std::ostream & operator<< ( std::ostream & salida , const QMat & a );


		//auxiliar
		T maximumElement();
		T minimumElement();
		// Creates a new gsl_matrix * from a matrix
		operator gsl_matrix * () const
		{
			gsl_matrix *result = gsl_matrix_alloc(rows, cols);
			for(int i = 0; i < rows; i++)
			  for(int j = 0; j < cols; j++)
				gsl_matrix_set(result, i, j, this->operator()(i, j));
			return result;
		}

// 		operator Eigen::MatrixXf ()
// 		{
// 			Eigen::MatrixXf  m = Eigen::Map<Eigen::MatrixXf> (this->getWriteData(),rows,cols);
// 			return m;
// 		}
	};
};

#include "qvec.h"
#endif

