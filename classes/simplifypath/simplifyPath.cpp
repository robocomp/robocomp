#include <string>
#include "simplifyPath.h"

using std::vector;
//Given an array of points, "findMaximumDistance" calculates the GPS point which have largest distance from the line formed by first and last points in RDP algorithm. Returns the index of the point in the array and the distance.

const std::pair<int, double> simplifyPath::findMaximumDistance(const vector<Point> &Points) const
{
	Point firstpoint = Points[0];
	Point lastpoint = Points[Points.size() - 1];
	int index = 0;  //index to be returned
	double Mdist = -1; //the Maximum distance to be returned

	//distance calculation
	Point p = lastpoint - firstpoint;
	for (uint i = 1; i < Points.size() - 1; i++)
	{ //traverse through second point to second last point
		Point pp = Points[i] - firstpoint;
		double Dist = fabs(pp * p) / p.Norm(); //formula for point-to-line distance
		if (Dist > Mdist)
		{
			Mdist = Dist;
			index = i;
		}
	}
	return std::make_pair(index, Mdist);
}


vector<Point> simplifyPath::simplifyWithRDP(vector<Point> &Points, double epsilon) const
{
	if (Points.size() < 3)
	{  //base case 1
		return Points;
	}
	std::pair<int, double> maxDistance = findMaximumDistance(Points);
	if (maxDistance.second >= epsilon)
	{
		int index = maxDistance.first;
		vector<Point>::iterator it = Points.begin();
		vector<Point> path1(Points.begin(), it + index + 1); //new path l1 from 0 to index
		vector<Point> path2(it + index, Points.end()); // new path l2 from index to last

		vector<Point> r1 = simplifyWithRDP(path1, epsilon);
		vector<Point> r2 = simplifyWithRDP(path2, epsilon);

		//Concat simplified path1 and path2 together
		vector<Point> rs(r1);
		rs.pop_back();
		rs.insert(rs.end(), r2.begin(), r2.end());
		return rs;
	}
	else
	{ //base case 2, all points between are to be removed.
		vector<Point> r(1, Points[0]);
		r.push_back(Points[Points.size() - 1]);
		return r;
	}
}







