//
// Created by robolab on 4/09/18.
//

#ifndef PROJECT_DOUBLEBUFFERCONVERTERS_H
#define PROJECT_DOUBLEBUFFERCONVERTERS_H

//TODO: This implementantions should go it's own file
class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
{
public:
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
    {
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData)
    {
        return false;
    }
};
//class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
//{
//public:
//    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
//    {
//        if (iTypeData.is_valid())
//        {
//
//            //            this->resize(d.width() * d.height()*data_size);
//            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
//            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
//            return true;
//        }
//        return false;
//    }
//
//    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData)
//    {
//        return false;
//    }
//};

class FloatSeqConverter : public Converter<astra::DepthFrame, RoboCompRGBD::DepthSeq>
{
public:
    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData)
    {
        if (iTypeData.is_valid())
        {
            std::copy(&iTypeData.data()[0], &iTypeData.data()[0]+(iTypeData.width()*iTypeData.height()), std::begin(oTypeData));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::DepthSeq &oTypeData, astra::DepthFrame &iTypeData)
    {
        return false;
    }
};



//class FloatSeqConverter : public Converter<astra::DepthFrame, RoboCompRGBD::DepthSeq>
//{
//public:
//    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData)
//    {
//        if (iTypeData.is_valid())
//        {
//
//            //            this->resize(d.width() * d.height()*data_size);
//            std::copy(&iTypeData.data()[0], &iTypeData.data()[0]+(iTypeData.width()*iTypeData.height()), std::end(oTypeData));
//            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
//            return true;
//        }
//        return false;
//    }
//
//    bool OtoI(const RoboCompRGBD::DepthSeq &oTypeData, astra::DepthFrame &iTypeData)
//    {
//        return false;
//    }
//};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
