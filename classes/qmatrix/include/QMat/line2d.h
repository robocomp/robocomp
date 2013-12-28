/*
    The contents of this file are subject to the Mozilla Public License
    Version 1.1 (the "License"); you may not use this file except in
    compliance with the License. You may obtain a copy of the License at
    http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS IS"
    basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
    License for the specific language governing rights and limitations
    under the License.

    The Original Code is ______________________________________.

    The Initial Developer of the Original Code is ________________________.
    Portions created by ______________________ are Copyright (C) ______
    _______________________. All Rights Reserved.

    Contributor(s): ______________________________________.

    Alternatively, the contents of this file may be used under the terms
    of the _____ license (the  "[___] License"), in which case the
    provisions of [______] License are applicable instead of those
    above.  If you wish to allow use of your version of this file only
    under the terms of the [____] License and not to allow others to use
    your version of this file under the MPL, indicate your decision by
    deleting  the provisions above and replace  them with the notice and
    other provisions required by the [___] License.  If you do not delete
    the provisions above, a recipient may use your version of this file
    under either the MPL or the [___] License."
*/


#ifndef LINE2D_H
#define LINE2D_H

#include <QtCore>
#include <limits>
#include <boost/iterator/iterator_concepts.hpp>
#include <QMat/qvec.h>

using namespace RMat;

class Line2D : public QVec
{

public:
	Line2D();
	Line2D(T x1, T y1, T x2, T y2);
	Line2D(const QVec &dir, T x1, T y1);
	Line2D(T A, T B, T C);
	Line2d(T slope, T intercept);
	Line2D(T slope, T x1, T x2);
	QVec dirVector();
	QVec perpVector();
	Qvec parallelVector();
	Line2D perpThroughPoint(T x1, T y1);
	Line2D parallelThroughPoint(T x1, T y1);
	T getSlope();
	T getIntercept();
	T perpDistanceToPoint();
	Line2D(const Line2D& other);
	virtual ~Line2D();
	virtual Line2D& operator=(const Line2D& other);
	virtual bool operator==(const Line2D& other) const;
};

#endif // LINE2D_H
















