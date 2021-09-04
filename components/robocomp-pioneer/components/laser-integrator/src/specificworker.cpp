/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    // read laser's extrinsics and intrinsics
    Eigen::Matrix3f front_rot;
    front_rot = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
               * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
    front_extrinsics.translate(Eigen::Vector3f(0.0, 200, 0.0));
    front_extrinsics.rotate(front_rot);
    std::cout << front_extrinsics.matrix() << std::endl;
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    // read both lasers
    try
    {  laser_list["front"] = std::make_tuple(laser_proxy->getLaserData(), front_extrinsics);}
    catch(const Ice::Exception &e)
    { std::cout << e.what() << std::endl;}
    try
    { laser_list["back"] = std::make_tuple(laser1_proxy->getLaserData(), back_extrinsics);}
    catch(const Ice::Exception &e)
    { std::cout << e.what() << std::endl;}

    // compute the 360 laser
    auto res = merge(laser_list);
    draw_laser(res);
    fps.print("FPS: ");

    std::scoped_lock lock(my_mutex);
    new_laser.swap(res);
}

////////////////////////////////////////////////////////////
RoboCompLaser::TLaserData SpecificWorker::merge(const std::map<std::string, Laser> &laser_list)
{
    // create 350 laser
    const int MAX_LASER_BINS = 360;
    std::vector<std::set<float>> bins(MAX_LASER_BINS);
    RoboCompLaser::TLaserData new_laser(MAX_LASER_BINS, {0.0, 0.0});

    // go through all the lasers
    for (const auto &[name, laser]: laser_list)
    {
        auto &[ldata, extrinsics] = laser;
        for (const auto &[dist, angle]: ldata)
        {
            // convert polar data to cartesian
            Eigen::Vector3f measure{dist * sin(angle), dist * cos(angle), extrinsics.translation().z()};
            // transform (x,y) point to origin's coordinate system using the laser's extrinsics
            auto new_vector = extrinsics * measure;
            // compute angle with robot's Y axos
            float new_angle = atan2(new_vector.x(), new_vector.y());
            // convert to degrees and cast to int
            int degrees = (int)(new_angle * (180.0/M_PI));
            // insert in the corresponding bin the distance from the origin. Insertions are sorted
            bins[degrees].emplace(new_vector.norm());
        }
    }
    // recover the first/minimum elements of each bien
    for(const auto &&[i, ray] : iter::enumerate(bins))
    {
        new_laser[i].dist = *ray.cbegin();
        float angle = i * M_PI / 180.0;
        if(i > 180)
            angle = -angle;
        new_laser[i].angle = angle;
    }
    return new_laser;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if(ldata.empty()) return;

    const int lado = 800;
    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
    laser_img = cv::Scalar(255,255,255);
    float scale = 100;
    float x = ldata.front().dist * sin(ldata.front().angle) * scale + lado/2;
    float y = lado - ldata.front().dist * cos(ldata.front().angle) * scale;
    cv::line(laser_img, cv::Point{lado/2,lado}, cv::Point(x,y), cv::Scalar(0,200,0));
    for(auto &&l : iter::sliding_window(ldata, 2))
    {
        int x1 = l[0].dist * sin(l[0].angle) * scale + lado/2;
        int y1 = 500 - l[0].dist * cos(l[0].angle) * scale;
        int x2 = l[1].dist * sin(l[1].angle) * scale + lado/2;
        int y2 = 500 - l[1].dist * cos(l[1].angle) * scale;
        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
    }
    x = ldata.back().dist * sin(ldata.back().angle) * scale + lado/2;
    y = lado - ldata.back().dist * cos(ldata.back().angle) * scale;
    cv::line(laser_img, cv::Point(x,y), cv::Point(lado/2,lado), cv::Scalar(0,200,0));

    cv::imshow("Laser 360", laser_img);
    cv::waitKey(2);
}
////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    return RoboCompLaser::TLaserData();
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    return RoboCompLaser::LaserConfData();
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    std::lock_guard<std::mutex> lg(my_mutex);
    return new_laser;
}

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser1_proxy->getLaserAndBStateData(...)
// this->laser1_proxy->getLaserConfData(...)
// this->laser1_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

