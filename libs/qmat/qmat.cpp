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
#ifdef COMPILE_IPP
#include <qmat/ippWrapper.h>
#endif

#include <qmat/qmat.h>
#include <qmat/qvec.h>

using namespace RMat;


/**
 * \brief Default constructor
 *
 * By default, it constructs a 1x1 matrix
*/
QMat::QMat() : cols(1), rows(1)
{
	data = new DataBuffer(cols*rows);
}
/**
 * \brief Copy constructor
 * @param A matrix to be copied
*/
QMat::QMat ( const QMat &A )
{
	cols = A.cols;
	rows = A.rows;
	data = A.data;
}
/**
 * \brief Dimensional constructor 
 * @param rows Number of rows 
 * @param cols Number of cols
*/
QMat::QMat ( int rows, int cols) : cols(cols), rows(rows)
{
	data = new DataBuffer(cols*rows);
}
/**
 * \brief Dimensional constructor with especific valor
 *
 * Creates a new matrix of the specified dimensions, and with each one of its cells containing a provided value.
 * @param rows Number of rows 
 * @param cols Number of cols
 * @param val Default value to set the elements of the matrix
*/
QMat::QMat ( int rows, int cols, const T &val ) : cols(cols), rows(rows)
{
	data = new DataBuffer(cols*rows);
	set( val );
}

/**
 * \brief Default constructor for column vector
 *
 * It constructs a rowsx1 column vector
 * @param rows Number of rows
*/
QMat::QMat ( int rows ) : cols(1), rows(rows)
{
	data = new DataBuffer(rows);
}

/**
 * \brief Vector to matrix constructor
 *
 * Creates a row or column matrix from vector
 * @param vector Vector to convert to matrix type
 * @param rowVector if true, creates a row matrix from the vector, else creates a column matrix from it
*/
QMat::QMat(const QVec &vector, const bool rowVector)
{
	cols = rowVector?vector.size():1;
	rows = rowVector?1:vector.size();
	
	data = new DataBuffer(cols*rows);
	
	if (rowVector)
		setRow(0, vector);
	else
		setCol(0, vector);
}
/**
 * \brief Constructor from a list of QVec
 *
 * Creates a new matrix from a list of row QVec
 * @param vectorList List of row vectors
*/
QMat::QMat(const QList<QVec> &vectorList): cols(vectorList.at(0).size()), rows(vectorList.size())
{
	data = new DataBuffer(cols*rows);
	
	for (int n = 0; n < getRows(); n++)
	{
		Q_ASSERT(vectorList.at(n).size() == getCols());
		setRow(n, vectorList.at(n));
	}
}
/**
 * \brief Constructor from a gsl matrix
 *
 * Creates a new matrix from a gsl matrix
 * @param matrix matrix to  be copied
*/
QMat::QMat(const gsl_matrix *matrix): cols(matrix->size2), rows(matrix->size1)
{
	data = new DataBuffer(cols*rows);
	
	for(int i = 0; i < rows; i++)
	  for(int j = 0; j < cols; j++)
		this->operator()(i, j) = gsl_matrix_get(matrix, i, j);
}
/**
 * \brief Fill matrix with specific value
 * @param v Especific value to fill the matrix
*/
void QMat::set ( T v )
{
	T *resultData = getWriteData();
	const int dataSize = getDataSize();
	for (int i = 0; i < dataSize; i++)
	    resultData[i] = v;
}
/**
 * \brief Matrix-matrix equality operator
 *
 * @param A matrix for compare operation
 * @return bool true if given matrix has same dimensions and equal content, othe case false
*/
bool QMat::operator== (const QMat & A)
{
	if (rows != A.rows) return false;
	if (cols != A.cols) return false;

	for(int r=0;r<rows;r++)
		for(int c=0;c<cols;c++)
			if(A(r,c) != this->operator()(r,c))
				return false;
	return true;
}
/**
 * \brief Copy operator from specific value 
 *
 * Fill matrix with specific value
 * @param a Especific value
*/
QMat & QMat::operator= ( T a )
{
	set ( a );
	return *this;
}
/**
 * \brief Copy operator from matrix 
 *
 * Copy the given matrix
 * @param A Matrix to be copied
*/
QMat & QMat::operator = ( const  QMat & A )
{
	data = A.data;
	rows = A.rows;
	cols = A.cols;
	return *this;
}
/**
 * \brief Copy function
 *
 * Return a copy of this matrix
 * @return QMat matrix copied
*/
QMat QMat::copy()
{
	QMat A (rows, cols);
#ifdef COMPILE_IPP
	ippmCopy_ma_32f_SS ( toData(), rows*cols*sizeof ( T ), cols*sizeof ( T ), sizeof ( T ), A.toData(),rows*cols*sizeof ( T ), cols*sizeof ( T ), sizeof ( T ),  cols, rows, 1 );
#else
	memcpy(A.toData(), toData(), rows*cols*sizeof(T));
#endif
	return A;
}

/**
 * \brief Inject function
 *
 * Inject given matrix into this matrix at given position
 * @param A matrix to be injected
 * @param roff row offset
 * @param coff column offset
 * @return QMat& matrix reference with given matrix injected
*/
QMat & QMat::inject ( const QMat & A, const int roff, const int coff )
{
	if ( (rows < A.nRows()+roff) or (cols < A.nCols()+coff) )
	{
		throw "QMat::inject inject destination matrix not big enough";
	}
#ifdef COMPILE_IPP
	ippmCopy_ma_32f_SS ( A.toDataConst(), A.nRows() *A.nCols() *sizeof ( T ), A.nCols() *sizeof ( T ), sizeof ( T ), toData() + ( roff*cols+coff ),rows*cols*sizeof ( T ), cols*sizeof ( T ), sizeof ( T ),  A.nCols(), A.nRows(), 1 );
#else
 	for(int r=0; r<A.rows; r++)
 		for(int c=0; c<A.cols; c++)
 			operator()(r+roff, c+coff) = A(r,c);
#endif
	return *this;
}
/**
 * \brief Matrix Addition operator: \f$ C = this + A \f$
 * @param A Matrix addend
 * @return QMat	Operation result
 */
QMat QMat::operator+ ( const QMat & A ) const
{
	Q_ASSERT ( equalSize ( *this, A ) );
	QMat C ( rows,cols );
#ifdef COMPILE_IPP
	ippmAdd_mm_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), A.toDataConst(), cols*sizeof ( T ), sizeof ( T ), C.toData(), cols*sizeof ( T ), sizeof ( T ), cols, rows );
#else
 	for(int r=0; r<A.rows; r++)
 		for(int c=0; c<A.cols; c++)
 			C(r, c) = A(r,c) + operator()(r, c);
#endif	
	return C;
}
/**
 * \brief Matrix Subtraction operator: \f$ C = this - A \f$
 * @param A Matrix subtrahend
 * @return QMat Operation result
 */
QMat QMat::operator- ( const QMat & A ) const
{
	Q_ASSERT ( equalSize ( *this, A ));
	QMat C ( rows,cols );
#ifdef COMPILE_IPP
	ippmSub_mm_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), A.toDataConst(), cols*sizeof ( T ), sizeof ( T ), C.toData(), cols*sizeof ( T ), sizeof ( T ), cols, rows );
#else
 	for(int r=0; r<A.rows; r++)
 		for(int c=0; c<A.cols; c++)
 			C(r, c) = A(r,c) - operator()(r, c);
#endif	
	return C;
}
/**
 * \brief Element to element product operator; \f$ C = this * A \f$
 * @param A Matrix factor for operation
 * @return QMat Operation result
 */
QMat QMat::operator& ( const QMat & A ) const
{
	Q_ASSERT(rows == A.rows);
	Q_ASSERT(cols == A.cols);
	
	QMat C ( rows,cols );
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ ) 
			C ( i,j ) =  this->operator()(i,j) * A ( i,j );
	return C;
}
/**
 * \brief Element to specific value product operator; \f$ C = this * f \f$
 *
 * Matrix element multiplication with especific value 
 * @param f Specific value for operation
 * @return QMat New matrix result
 */
QMat  QMat::operator& ( const T f ) const
{
	QMat C ( rows,cols );
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ ) C ( i,j ) =  this->operator()(i,j) * f;
	return C;
}
/**
 * \brief Element to specific value product operator (inplace);\f$ this = this * f \f$
 *
 * Matrix element multiplication with especific value 
 * @param f Specific value for operation
 * @return QMat& this matrix result
 */
QMat & QMat::operator&= ( const T &f )
{
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ ) this->operator()(i,j) *= f;
	return *this;
}
/**
 * \brief Element to element product (inplace); \f$ this = this * A \f$
 * @param A matrix for the operation
 * @return QMat& this matrix resul
 */
QMat & QMat::operator&= ( const QMat & A )
{
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ ) 
			this->operator()(i,j) *= A ( i,j );
	return *this;

}
/**
 * \brief Matrix to specific value product operator; \f$ C = this * f \f$
 * @param f Specific value factor for operation
 * @return QMat New matrix result
 */
QMat RMat::QMat::operator *( const T f) const
{
	QMat C ( rows,cols );
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )	C ( i,j ) =  this->operator()(i,j) * f;
	return C;
}
/**
 * \brief Matrix to matrix product operator; \f$ C = this * A \f$
 *
 * IPP coge (columnas, filas) en las llamadas
 * @param A matrix factor for operation
 * @return QMat New matrix result
 */
QMat QMat::operator * ( const QMat & A ) const
{
// 	printf("Operator *: (%d,%d) x (%d,%d)\n", rows, cols, A.rows, A.cols);
	QMat C=zeros( rows, A.nCols() );
	if ( cols != A.nRows())
	{
		QString ex= "QMat::operator* - a.cols!=b.rows";
		throw ex;
	}
	else
	{
#ifdef COMPILE_IPP
	ippmMul_mm_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), cols, rows, A.toDataConst(), A.nCols() *sizeof ( T ), sizeof ( T ), A.nCols(), A.nRows(), C.toData(), C.nCols() *sizeof ( T ), sizeof ( T ) );
#else
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<A.cols;j++)
		{
			C(i,j)=0;
			for(int k=0;k<cols;k++)
			{
				C(i,j) += operator()(i,k)*A(k,j);
			}
// 			printf("%f ", C(i,j));
		}
// 		printf("\n");
	}
#endif
	}
	return C;
}
/**
 * \brief Matrix to vector product operator; \f$ C = this * vector \f$
 * @param A vector factor for operation
 * @return QVec New vector result
 */
QVec RMat::QMat::operator *(const QVec & vector) const
{
	Q_ASSERT ( cols == vector.size());
	QMat aux = vector.toColumnMatrix();
	QMat result = operator*(aux);
	return result.toVector();
}
/**
 * \brief Matrix to specific value substraction operator; \f$ C = this - f \f$
 * @param f Specific value factor for operation
 * @return QMat New matrix result
 */
QMat RMat::QMat::operator -(const T f) const
{
	QMat C ( rows,cols );
	for ( int i=0; i<rows; i++ )
	{
		for ( int j=0; j<cols; j++ )
		{
			C ( i,j ) =  this->operator()(i,j) - f;
		}
	}
	return C;
}
/**
 * \brief Matrix to matrix division operator; \f$ C = this / A \f$
 * @param A matrix factor for operation
 * @return QMat New matrix result
 */
QMat QMat::operator/ ( const QMat & A ) const
{
	Q_ASSERT ( equalSize ( *this, A ));
	QMat C (rows,cols);
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ ) C ( i,j ) =  this->operator()(i,j) / A ( i,j );
	return C;
}
/**
 * \brief Matrix to specific value division operator; \f$ C = this / f \f$
 * @param f Specific value factor for operation
 * @return QMat New matrix result
 */
QMat QMat::operator/ ( const T f ) const
{
	QMat C ( rows,cols );
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )	C ( i,j ) =  this->operator()(i,j) / f;
	return C;
}
/**
 * \brief Matrix Addition operator (inplace): \f$ this = this + A \f$
 * @param A Matrix addend
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator+= ( const QMat & A )
{
	Q_ASSERT ( equalSize ( *this, A ));
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )
			this->operator()(i,j) += A ( i,j );
	return *this;
}
/**
 * \brief Matrix Addition operator (inplace): \f$ this = this + f \f$
 * @param f Specific value factor for operation
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator+= ( const T &f )
{
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )
			this->operator()(i,j) += f;
	return *this;
}
/**
 * \brief Matrix subtraction operator (inplace): \f$ this = this + A \f$
 * @param A Matrix subtrahend
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator-= ( const QMat & A )
{
	Q_ASSERT ( equalSize ( *this, A ));
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )
			this->operator()(i,j) -= A ( i,j );
	return *this;
}
/**
 * \brief Matrix subtraction operator (inplace): \f$ this = this - f \f$
 * @param f Specific value subtrahend
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator-= ( const T &f )
{
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )
			this->operator()(i,j) -= f;
	return *this;
}
/**
 * \brief Matrix to matrix division operator (inplace); \f$ this = this / A \f$
 * @param A matrix factor for operation
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator/= ( const QMat & A )
{
	Q_ASSERT ( equalSize ( *this, A ));
	for ( int i=0; i<rows; i++ )
	{
		for ( int j=0; j<cols; j++ )
		{
			this->operator()(i,j) /= A ( i,j );
		}
	}
	return *this;
}
/**
 * \brief Matrix to specific value division operator (inplace); \f$ this = this / f \f$
 * @param f Specific value factor for operation
 * @return  QMat& this matrix resul
 */
QMat & QMat::operator/= ( const T &f )
{
	for ( int i=0; i<rows; i++ )
	{
		for ( int j=0; j<cols; j++ )
		{
			this->operator()(i,j) /= f;
		}
	}
	return *this;
}

std::ostream & operator<< ( std::ostream & out, const QMat & A )
{
	const int cols = A.nCols() , rows = A.nRows();
	const RMat::T *data = A.toDataConst();

	out << "RMat::QMat (" << rows << ", " << cols << ")" << std::endl;
	out << "[" << std::endl;

	for ( int i=0; i < rows ; i++ )
	{
		out << "    [ ";
		for ( int j=0; j < cols; j++ )
			out << qPrintable ( QString ( "%1" ).arg ( data[i*cols + j], -8, 'f', 6 ) ) << " ";
		out << "]" << std::endl;
	}
	out << "]" << std::endl;
	return out;

}

void QMat::print ( const QString & s ) const
{
	std::cout << qPrintable ( s ) << "(" << rows << ", " << cols << ")" << std::endl;
	std::cout << "[" << std::endl;

	for ( int i=0; i < rows ; i++ )
	{
		std::cout << "     ";
		for ( int j=0; j < cols; j++ )
			std::cout << qPrintable ( QString ( "%1" ).arg ( this->operator()(i,j) , -8, 'f', 6 ) ) << " ";
		std::cout << ";" << std::endl;
	}
	std::cout << "]" << std::endl;
}
/**
 * \brief Construct a diagonal matrix with specific value (inplace)
 * @param d Specific value for diagonal positions
 */
void QMat::makeDiagonal ( T d )
{
	Q_ASSERT_X ( rows == cols, "makeDiagonal" , "Not square matrix!" );
	for (int i=0; i<rows; i++)
	{
		for (int j=0; j<cols; j++)
		{
			this->operator()(i,j) = 0;
		}
	}
	for ( int i=0; i<rows; i++ )
		this->operator()(i,i) = d;
}
/**
 * \brief Construct a diagonal matrix 
 *
 * Use diagonal values to create a new diagonal matrix
 * @param d Matrix to get diagonal values
 * @return QMat new diagonal matrix
 */
QMat QMat::diagonal ( const QMat & d )
{
	QMat R ( d.nRows(),d.nRows() );
	for (int i=0; i<d.nRows(); i++)
	{
		for (int j=0; j<d.nCols(); j++)
		{
			R(i,j) = 0;
		}
	}
	for ( int i=0; i < d.nRows(); i++ )
		R ( i,i ) = d ( i );
	return R;
}
/**
 * \brief Construct a diagonal matrix 
 *
 * Use vector values to create a new diagonal matrix
 * @param v Vector to get diagonal values
 * @return QMat new diagonal matrix
 */
QMat RMat::QMat::makeDiagonal ( const QVec &v )
{
	QMat R ( v.size(),v.size(), (T)0 );
	int f = v.size();
	for ( int i=0; i<f; i++ )
		R ( i,i ) = v ( i );
	return R;
}
/**
 * \brief Gets the diagonal of the matrix 
 * @return QVec new vector with diagonal matrix values
 */
QVec RMat::QMat::getDiagonal( )
{
	Q_ASSERT( isSquare() );
	QVec v ( rows );
	for ( int i=0; i<rows; i++ )
			v ( i ) = this->operator()(i,i);
	return v;
}
/**
 * \brief Change the order of the indexes in the matrix \f$ C = this^t \f$
 * @return QMat new matrix with indexes reordered
 */
QMat QMat::transpose( ) const
{
	QMat C ( cols, rows );
#ifdef COMPILE_IPP
	ippmTranspose_m_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), cols, rows, C.toData(), rows*sizeof ( T ), sizeof ( T ) );
#else
	for (int fila=0; fila<rows; fila++ )
		for ( int columna=0; columna<cols; columna++ )
			C(columna,fila) = operator()(fila,columna);
#endif
	return C;
}
/**
 * \brief Obtains the determinant of a squared matrix
 *
 * For matrix size smaller 4 direct operation
 * @return T determinant value
 */
RMat::T QMat::determinant( ) const
{
	Q_ASSERT( isSquare() );
	T R = 1.f;
	if(cols==1)
		R = this->operator()(0,0);
	else if(cols==2)
		R = this->operator()(0,0)*this->operator()(1,1)-(this->operator()(0,1)*this->operator()(1,0));
	else if(cols==3)
	{
		R = (this->operator()(0,0)*this->operator()(1,1)*this->operator()(2,2)) + 
			(this->operator()(0,1)*this->operator()(1,2)*this->operator()(2,0)) +
			(this->operator()(0,2)*this->operator()(1,0)*this->operator()(2,1));
		R -=(this->operator()(0,2)*this->operator()(1,1)*this->operator()(2,0)) + 
			(this->operator()(0,1)*this->operator()(1,0)*this->operator()(2,2)) +
			(this->operator()(0,0)*this->operator()(1,2)*this->operator()(2,1));
	}
	else
	{
#ifdef COMPILE_IPP
		T pBuffer[cols*cols+cols];
		ippmDet_m_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), cols, pBuffer, &R );
#else
		gsl_matrix *m = *this;
		int sign;
		gsl_permutation *p = gsl_permutation_alloc(this->nRows());
		gsl_linalg_LU_decomp(m, p, &sign);
		R = (float)gsl_linalg_LU_det(m, sign);
		gsl_matrix_free(m);
		gsl_permutation_free(p);
#endif
	}
	return R;
}
/**
 * \brief Gets the trace of the matrix
 *
 * The trace of the matrix is the sum of the diagonal elements
 * \f$ \mathrm{tr}(A) = a_{11} + a_{22} + \dots + a_{nn}=\sum_{i=1}^{n} a_{i i} \f$ \n
 * where \f$ a_{ij} \f$ are the elements of the matrix \f$ A \f$. 
 * @return T trace value
 */
RMat::T QMat::trace( )
{
	T R = 0;
#ifdef COMPILE_IPP
	ippmTrace_m_32f ( toData(),cols*sizeof ( T ), sizeof ( T ), minDim(), &R );
#else
	for (int i=0; i<(cols<rows?cols:rows); ++i)
		R+=operator()(i,i);
#endif
	return R;
}


QMat QMat::invert( ) const
{
	Q_ASSERT( isSquare() );
	QMat R ( rows, cols );
#ifdef COMPILE_IPP
	T pBuffer[cols*cols+cols];
	IppStatus status = ippmInvert_m_32f ( getReadData(), cols*sizeof ( T ), sizeof ( T ), pBuffer, R.toData(), cols*sizeof ( T ), sizeof ( T ), cols );
#else
	int status =1;
	if(determinant()!=0)
	{
		status=0;
		gsl_matrix *u = *this;
		gsl_vector *s = gsl_vector_alloc (cols);
		gsl_matrix *v = gsl_matrix_alloc (cols,cols);
		gsl_vector *work = gsl_vector_alloc (cols);
		
		gsl_matrix *X = gsl_matrix_alloc (cols,cols);
		gsl_linalg_SV_decomp_mod(u, X, v, s, work);
		gsl_matrix_free(X);

		// pasamos de vuelta:
		QMat U = u;
		QMat V = v;
		gsl_matrix_free(u);
		gsl_matrix_free(v);
		gsl_vector_free(work);
		
		QMat S = QMat::zeros(rows,cols);
		for (int i = 0; i < cols; i++)
			S(i,i) = 1 / gsl_vector_get(s, i);

		gsl_vector_free(s);
		return V * S * U.transpose();
	}
#endif
	if ( status == 0 )
		return R;
	else
	{
		QString ex= "QMat::invert() - Singular matrix";
		throw ex;
	}
}

T RMat::QMat::vectorNormL2( ) const
{
	Q_ASSERT_X ( cols == 1 , "length" , "Not a column vector");

	T res = 0.;
	for ( int i=0; i < rows; i++ )
		res += ( *this ) ( i,0 ) * ( *this ) ( i,0 );
	return ::sqrt ( res );
}

void QMat::ones( )
{
	set ( 1. );
}

QMat QMat::ones ( int m, int n )
{
	QMat R ( m, n );
	R.ones();
	return R;
}
/**
 * \brief Creates an identity matrix (inplace)
 *
 * @return QMat& this matrix resul
 */

QMat & QMat::makeIdentity( )
{
	Q_ASSERT ( isSquare());
#ifdef COMPILE_IPP
		ippmLoadIdentity_ma_32f(toData(), rows*cols*sizeof(T), cols*sizeof(T), sizeof(T), cols, rows, 1);
#else
	for ( int i=0; i<rows; i++ )
		for ( int j=0; j<cols; j++ )
			operator()(i,j) = (i==j)?1:0;
#endif
	return *this;
}

QMat & RMat::QMat::makeUnitModulus()
{
	Q_ASSERT( isColVector() );
	T mod = vectorNormL2();
	if ( mod > 0. )
		( *this ) /= mod;
	return *this;
}

QMat QMat::identity(int m)
{
	QMat R (m, m);
	R.makeIdentity();
	return R;
}

//deprecated. Use getRow
QMat RMat::QMat::getFil ( int n )
{
	QMat v ( 1,cols );
	for ( int i=0;i< cols;i++ ) v ( 0,i ) = ( *this ) ( n,i );
	return v;
}

QMat RMat::QMat::sqrt( )
{
	int i;
	QMat R ( rows, cols );
	for ( i=0;i< rows*cols;i++ )
		R.getWriteData()[i] = ::sqrt ( fabs ( getReadData()[i]) );
	return R;
}

/**
 * \brief Obtains the Cholesky decomposition
 *
 * The Cholesky decomposition obtains a matrix L from an original matrix A, satisfying the following equation: \f$ A = L L^{*} \f$. \n
 * The matrix \f$ L \f$ is a lower triangular matrix, with strictly positive diagonal entries. The \f$ L^{*} \f$ is the conjugate transpose of the matrix \f$ L \f$. \n
 * Diagonal values from IPP come inverted. The method restores them to its original value.
 * @return QMat new matrix contains triangular superior values
 */
QMat RMat::QMat::cholesky( )
{
	Q_ASSERT( isSquare() );

	QMat R=zeros( rows,cols );
	//QMat T = makeDefPos();
#ifdef COMPILE_IPP
	IppStatus status = ippmCholeskyDecomp_m_32f ( toDataConst(), cols*sizeof ( T ), sizeof ( T ), R.toData(), cols*sizeof ( T ), sizeof ( T ), cols );
	if ( status == ippStsNoErr )
	{
		for ( int i=0; i<R.nCols(); i++ )
		{
			if ( fabs ( R ( i,i ) ) > 0. )
				R ( i,i ) = 1./R ( i,i );
			else R ( i,i ) =0.;
		}
		return R.transpose();
	}
	else
	{
		printf ( "Cholesky error: %d, %s\n", status, ippGetStatusString ( status ) );
		( *this ).print ( "this" );
		QString s="QMat::cholesky() - Error returned by IPP"; throw s;
	}
#else
// 	qFatal("cholesky using Eigen has not been properly tested yet");
	gsl_matrix *a = *this;
	gsl_linalg_cholesky_decomp(a);

	for (int i = 0; i < rows; i++)	
		for (int j = 0; j < cols; j++)
			if (j >= i)
				R(i,j) = gsl_matrix_get(a, i, j);
	gsl_matrix_free(a);
	return R;
#endif	
	
}

QMat RMat::QMat::eigenValsVectors ( QVec & vals )
{
	if ( isSquare() and ( vals.size() ==cols ) )
	{
		QMat R ( rows,cols,0.);
#ifdef COMPILE_IPP
			T valsIm[cols];
			int SizeBytes;
			ippmEigenValuesVectorsGetBufSize_32f(cols, &SizeBytes);
			Ipp8u pBuffer[SizeBytes];
			IppStatus status = ippmEigenValuesVectorsRight_m_32f (toData(), cols*sizeof(T), sizeof(T), R.toData(), cols*sizeof(T), sizeof(T), vals.getWriteData(),valsIm, cols, pBuffer); 

			if ( status == ippStsNoErr )
				return R;
			else
			{
				printf ( "EigenValsVectors error: %d, %s\n", status, ippGetStatusString ( status ) );
				( *this ).print ( "this" );
				QString s="QMat::eigenVectors() - Error returned by IPP"; throw s;
			}
#else
// 			qFatal("eigen values using Eigen has not been properly tested yet");
			double data[this->getDataSize()];
			const T *dataBuffer = this->getReadData();
			for(int i = 0; i < this->getDataSize(); i++)
				data[i] = (double)dataBuffer[i];

			gsl_matrix_view m = gsl_matrix_view_array (data, rows,rows);
			gsl_vector *eval = gsl_vector_alloc (rows);
			gsl_matrix *evec = gsl_matrix_alloc (rows, rows);

			gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (rows);
			gsl_eigen_symmv (&m.matrix, eval, evec, w);
			gsl_eigen_symmv_free (w);
			gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_DESC);

			for(int i=0;i<rows;i++)
				vals(i) = gsl_vector_get(eval,i);
			QMat P(evec);
			
			gsl_vector_free (eval);
			gsl_matrix_free (evec);

			return P;
#endif
	}
	QString s="QMat::EigenValsVectors() - Not Square Matrix"; throw s;
	return QMat();
}

void RMat::QMat::SVD(QMat & U, QMat & D, QMat & V)
{
#ifdef COMPILE_IPP
// 	double ippA[rows*cols];
	double ippU[rows*rows], ippD[rows*cols], ippV[cols*cols];
	int i;
	
	U = QMat(rows, rows);
	D = QMat(rows, cols);
	V = QMat(cols, cols);

// 	for(i=0; i<rows*cols;i++)
// 	  ippA[i] = toData()[i];
	
	
	for(i=0; i<rows*rows;i++)
	  U.toData()[i] = ippU[i];
	
	for(i=0; i<cols;i+=1)
	  D.toData()[i*(cols+1)] = ippD[i];
	
	for(i=0; i<cols*cols;i++)
	  V.toData()[i] = ippV[i];
#else
	qFatal("QMat does not support eigen svd");
#endif

}

QMat RMat::QMat::makeDefPos()
{
	QVec vals ( rows,0. );
	QMat V = eigenValsVectors ( vals );
	//V.print("autovectores originales");
	for ( int i=0; i< rows; i++ )
	{
		if ( vals ( i ) <= 0. )
			vals ( i ) = 0.0000001;
	}
	QMat DD = QMat::makeDiagonal ( vals );
	QMat R = ( V * ( DD * V.transpose() ) );

	return R;
}

QMat RMat::QMat::matSqrt( )
{
	Q_ASSERT( isSquare() );
	//QMat T = makeDefPos();
	QVec vals ( rows );
	QMat V = eigenValsVectors ( vals );
	//V.print("autovectores");
	QMat VI = V.invert();
	//VI.print("VI");
	QMat D = VI * ( ( *this ) * V );
	for ( int i=0; i<rows;i++ )
		for ( int j=0;j<cols;j++ )
			if ( i != j ) D ( i,j ) =0.;
			else D ( i,j ) = ::sqrt ( fabs ( D ( i,j ) ) );
	//D.print("D");
	QMat R = ( V * ( D * VI ) );
	return R;

}


/// Auxiliary functions

bool QMat::isSquare ( const QMat &A ) const
{
	return ( A.nRows() == A.nCols() );
}
bool QMat::isSquare() const
{
	return ( rows == cols );
}
int QMat::minDim ( const QMat &A )
{
	return qMin ( A.nRows(), A.nCols() );
}
int QMat::minDim( )
{
	return qMin ( rows, cols );
}
int QMat::maxDim ( const QMat &A )
{
	return qMax ( A.nRows(), A.nCols() );
}
int QMat::maxDim( )
{
	return qMax ( rows, cols );
}
bool QMat::equalSize ( const QMat & A, const QMat & B ) const
{
	return ( A.rows == B.rows and A.cols == B.cols );
}

QMat RMat::QMat::random ( const int fi, const int co )
{
	QMat res ( fi, co );
	for ( int i = 0; i < fi; i++ )
		for ( int j = 0; j < co; j++ )
			res ( i,j ) = ( double ) abs ( rand() ) / ( double ) std::numeric_limits<int>::max();
	return res;
}

QMat RMat::QMat::operator ^(const QMat & A)
{
	Q_ASSERT( is3ColumnVector() );
	Q_ASSERT( is3ColumnVector(A) );
	QMat res(3);
	res(0) = toDataConst()[1]*A(2)-toDataConst()[2]*A(1);
	res(1) = -(toDataConst()[0]*A(2)-toDataConst()[2]*A(0));
	res(2) = toDataConst()[0]*A(1)-toDataConst()[1]*A(0);
	return res;

}

bool RMat::QMat::is3ColumnVector(const QMat & A) const
{
	return (A.nRows() == 3 and A.nCols() == 1);
}

bool RMat::QMat::is3ColumnVector() const
{
	return (rows == 3 and cols == 1);
}

bool RMat::QMat::canAllocateRotationMatrix() const
{
	return (getCols()>=3 and getRows()>=3);
}

QMat RMat::QMat::vec3(T x, T y, T z)
{
	QMat R(3);
	R(0)=x;R(1)=y;R(2)=z;
	return R;
}


QMat RMat::QMat::gaussian(const int fi, const float mean, const float stdev)
{
	QMat R(fi,1);
	static unsigned int Seed = QTime::currentTime().msec();
#ifdef COMPILE_IPP
	ippsRandGauss_Direct_32f(R.toData(), fi, mean, stdev, &Seed);
#else
// 		qFatal("eigen values using Eigen has not been properly tested yet");
	  const gsl_rng_type * T;
	  gsl_rng_env_setup();
	  T = gsl_rng_default;
	  gsl_rng *r = gsl_rng_alloc (T);
	  gsl_rng_set(r,Seed);
	  for(int i=0;i<fi;i++)
		R(i,0) = gsl_ran_gaussian ( r,stdev)+mean;
#endif
  	return R;
}


QMat RMat::QMat::fromStdVector(const std::vector<T> & v)
{
	Q_ASSERT( v.size() > 0);
	
	QVec res(v);
	return res.toColumnMatrix();
}

QMat RMat::QMat::zeroes(const int m, const int n)
{
	QMat R ( m, n );
	R.set(0.);
	return R;
}

QMat RMat::QMat::zeros(const int m, const int n)
{
	QMat R ( m, n );
	R.set(0.);
	return R;
}

QMat RMat::QMat::toCrossProdForm() const
{
	Q_ASSERT( is3ColumnVector());
	QMat R (3,3);
	R(0,0) = 0.;
	R(0,1) = -getReadData()[2];
	R(0,2) = getReadData()[1];
	R(1,0) = getReadData()[2];
	R(1,1) = 0.;
	R(1,2) = -getReadData()[0];
	R(2,0) = -getReadData()[1];
	R(2,1) = getReadData()[0];
	R(2,2) = 0.;
	return R;
}

// Quizás pornerlo para mas tipos de vector, no solo columna
QVec RMat::QMat::toVector() const
{
	Q_ASSERT( cols == 1 and rows>0);
	
	QVec result( getCol(0) );
	return result;
}

// QVec RMat::QMat::extractAnglesR() const
// {
// 	Q_ASSERT_X(canAllocateRotationMatrix(),"QMat::extractAngles_R", "Invalid matrix size in QMAt::extractAngles");
// 	const float small = 0.00000001;
// 
// 	// Normal case. We check that atan2(sin(x)*cos(y)),  cos(x)*cos(y)) is not zero because cos(y) is zero
// 	if (fabs(operator()(0,2)) <= 1.-small)
// 	{
// 		float rx = atan2(-operator()(1,2), operator()(2,2));
// 		float rz = atan2(-operator()(0,1), operator()(0,0));
// 		float ry = atan2( operator()(0,2), operator()(0,0)/cos(rz));
// 		return QVec::vec3(rx, ry, rz);
// 	}
// 	else
// 	{
// 		float rx;
// 		float ry;
// 		float rz = 0;
// 		if (-operator()(0,2)<0)
// 		{
// 			ry = M_PI/2.;
// 			rx = atan2(operator()(1,0), operator()(1,1));
// 		}
// 		else
// 		{
// 			ry = -M_PI/2.;
// 			rx = atan2(-operator()(1,0), operator()(1,1));
// 		}
// 		return QVec::vec3(rx, ry, rz);
// 	}
// }


bool RMat::QMat::extractAnglesR2(QVec &a, QVec &b) const
{
	Q_ASSERT_X(canAllocateRotationMatrix(),"QMat::extractAngles_R2", "Invalid matrix size in QMAt::extractAngles");
	const float small = 0.00000001;

	// Normal case
	if (fabs(operator()(0,2)) <= 1.-small)
	{
		// a
		a(0) = atan2(-operator()(1,2), operator()(2,2));
		a(2) = atan2(-operator()(0,1), operator()(0,0));
		a(1) = atan2( operator()(0,2), operator()(0,0)/cos(a(2)));
		// b
		b(1) = M_PI - a(1);
		b(0) = atan2(-operator()(1,2)/cos(b(1)), operator()(2,2)/cos(b(1)));
		b(2) = atan2(-operator()(0,1)/cos(b(1)), operator()(0,0)/cos(b(1)));
		// return false: two solutions
		return false;
	}
	else
	{
		// First vector
		a(2) = 0;
		if (-operator()(0,2)<0)
		{
			a(1) = M_PI/2.;
			a(0) = atan2(operator()(1,0), operator()(1,1));
		}
		else
		{
			a(1) = -M_PI/2.;
			a(0) = atan2(-operator()(1,0), operator()(1,1));
		}
		// Second vector, just copy the first one because there are infinite, so there's no point actually....
		b = a;
		// return true: infinite cases
		return true;
	}
}

//-----------------------------------------------------------------------------
//			MÉTODO EN PRUEBAS
//-----------------------------------------------------------------------------
// Devuelve los ángulos de rotación sacados de la matriz de rotación en un vector
// de 6 ELEMENTOS: x1, y1, z1, x2, y2, z2 (ángulos y sus opuestos: signos comabiados) En el primer caso
// X, Y, Z, X, Y, Z (ángulos repetidos) segundo caso.
// DOCUMENTACIÓN: http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf
QVec RMat::QMat::extractAnglesR() const
{
	// Ten en cuenta: matriz transpuesta y con signos cambiados.
	QVec angulos;
	float x, y, z;
	float x1, x2, y1, y2, z1, z2;
	
	if (fabs(operator()(0,2)) > 1 || fabs(operator()(0,2)) < 1)
	{
		// ROTACION EN Y
		y1 = asin(operator()(0,2));
		y2 = M_PI-y1;
		// ROTACION EN X
		x1 = atan2((-operator()(1,2)/cos(y1)), (operator()(2,2)/cos(y1)));
		x2 = atan2((-operator()(1,2)/cos(y2)), (operator()(2,2)/cos(y2)));
		//ROTACION EN Z
		z1 = atan2((-operator()(0,1)/cos(y1)), (operator()(0,0)/cos(y1)));
		z2 = atan2((-operator()(0,1)/cos(y2)), (operator()(0,0)/cos(y2)));
		angulos.push_back(x1);  angulos.push_back(y1);  angulos.push_back(z1); // ir por el camino 1
		angulos.push_back(x2);  angulos.push_back(y2);  angulos.push_back(z2); // ir por el camino 2
	}
	else
	{
		// REVISAR LOS SIGNOS!!!
		z = 0;
		if (operator()(0,2) == 1)//Original if -sin(y)==-1 en el nuestro: if sin(y)==1
		{
			y = M_PI/2;
			x = z + atan2( operator()(1,0), -operator()(2,0));// al final queda atan2(sin x, -(-cosx))-->atan2(sin x, cos x)
		}
		else //si sin(y)==-1 --> y = -pi/2
		{
			y = -M_PI/2;
			x = -z +atan2(-operator()(1,0), operator()(2,0));
		}
		angulos.push_back(x);  angulos.push_back(y);  angulos.push_back(z);
		angulos.push_back(x);  angulos.push_back(y);  angulos.push_back(z); //Repetimos los ángulos, por ser coherentes....
	}
	return angulos;
}

QVec RMat::QMat::extractAnglesR_min() const
{
	QVec r = extractAnglesR();
	QVec v1 = r.subVector(0,2);
	QVec v2 = r.subVector(3,5);
	if (v1.norm2() < v2.norm2())
		return v1;
	return v2;	
}




/// Access operations

//Do it with inject
void RMat::QMat::setCol(const int col, QVec vector)
{
	Q_ASSERT(col < nCols());
	Q_ASSERT(nRows() == vector.size());
 
	const int rows = nRows();
	for (int row= 0; row < rows; row++)
		operator()(row,col) = vector[row];

}

void RMat::QMat::setRow(const int row, QVec vector)
{
	Q_ASSERT(row < nRows());
	Q_ASSERT(nCols() == vector.size());
 
	const int cols = nCols();
	for (int col= 0; col < cols; col++)
		operator()(row,col) = vector[col];
}

const QVec RMat::QMat::getCol(const int col) const
{
 	Q_ASSERT(col < nCols());
 
 	const int rows = nRows();
	QVec result(rows);
	for (int row= 0; row < rows; row++)
		result[row] = operator()(row,col);
 
	return result;

}

void RMat::QMat::setRow(const int row, QVector< T > vector)
{
	Q_ASSERT(row < nRows());
	Q_ASSERT(nCols() == vector.size());
 
	const int cols = nCols();
	for (int col= 0; col < cols; col++)
		operator()(row,col) = vector[col];
}

const QVec RMat::QMat::getRow(const int row) const
{
	Q_ASSERT(row < nRows());
 
 	const int cols = nCols();
	QVec result(cols);
	for (int col= 0; col < cols; col++)
		result[col] = operator()(row,col);
 
	return result;
}

const QMat QMat::getSubmatrix(const int firstRow, const int lastRow, const int firstCol, const int lastCol) const
{
	Q_ASSERT(firstRow >= 0);
	Q_ASSERT(firstCol >= 0);
	Q_ASSERT(firstRow <= lastRow);
	Q_ASSERT(firstCol <= lastCol);
	Q_ASSERT(lastRow < nRows());
	Q_ASSERT(lastCol < nCols());

	QMat result(lastRow - firstRow +1, lastCol - firstCol +1);
#ifdef COMPILE_IPP
	ippmCopy_ma_32f_SS (getReadData()+firstRow*cols+firstCol,rows*cols*sizeof(T),cols*sizeof(T),sizeof(T),result.getWriteData(),result.nRows()*result.nCols()*sizeof (T), result.nCols()*sizeof(T),sizeof(T),lastCol-firstCol+1,lastRow-firstRow+1,1);
#else
	for (int row = 0; row <= lastRow-firstRow; row++)
		for (int col = 0; col <= lastCol-firstCol; col++)
			result(row, col) = operator()(row+firstRow, col+firstCol);
#endif
	return result;
}

/**
 * \brief Constructs am affine transformation matrix that converts N, D dimensional vectors. The affine conversion of each one is defined
 * with two points satisfying the line equation of the transformation: \f$ (x1,y1) -> (x2,y2) \f$ in each dimension.
 * The matrix can be used to transform magnitudes betweed different ranges. \n
 * The method computes the following line: \f$  y = x( (y2-y1)/(x2-x1) ) - x1( (y2-y1)/(x2-x1) ) + y1 \f$
 * where \f$ x1 = First.x(), y1 = Second.x(), x2 = First.y(), y2 = Second.y() ) \f$ \n
 * Multiply any vector augmented with an extra row set to 1, by the provided matrix to obtain the desired linear conversion
 * @param intervals List of QPairs containing each two intervals as QPointF: the first one for origin and the second one for image
 * @return QMat of dimensions intervals.size() x intervals.size()+1
 */
QMat RMat::QMat::afinTransformFromIntervals(const QList< QPair < QPointF , QPointF > > & intervals)
{
	int dim = intervals.size();
	Q_ASSERT(dim < MAX_DIMENSION);
	QMat res = QMat::zeros(dim,dim+1);
	for (int i=0; i<dim; i++)
	{
		QPointF pX = intervals[i].first;
		QPointF pY = intervals[i].second;
		//slant
		res(i,i) = (pY.y()-pY.x())/(pX.y()-pX.x()) ;
		//Disp
		res(i,dim) =   pY.x() - pX.x() * ((pY.y()-pY.x())/(pX.y()-pX.x())) ;
	}
	return res;
}

T QMat::maximumElement()
{
	T max=this->operator()(0,0);
	for(int i=0;i<rows;i++)
		for(int j=0;j<cols;j++)
			if(max < this->operator()(i,j))
				max = this->operator()(i,j);
	return max;
}

T QMat::minimumElement()
{
	T min=this->operator()(0,0);
	for(int i=0;i<rows;i++)
		for(int j=0;j<cols;j++)
			if(min > this->operator()(i,j))
				min = this->operator()(i,j);
	return min;
}

