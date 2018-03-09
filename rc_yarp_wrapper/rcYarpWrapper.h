#ifndef RCYARPWRAPPER_H
#define RCYARPWRAPPER_H

#include <rc_genicam_api/system.h>
#include <rc_genicam_api/interface.h>
#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/image.h>
#include <rc_genicam_api/config.h>

#include <rc_genicam_api/pixel_formats.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <yarp/os/all.h>
//#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include "rcYarpWrapper_IDL.h"

class rcYarpWrapper : public yarp::os::RFModule, public rcYarpWrapper_IDL
{
protected:
    double                      period;
    std::string                 name;                               //!< module name
    int                         scale;                              //!< scale factor to resize original image
    yarp::os::RpcServer         rpcPort;                            //!< rpc server to receive user request
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > port_mono, port_conf;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > port_disp;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > port_color;
    std::shared_ptr<rcg::Device>                dev;
    std::vector<std::shared_ptr<rcg::Stream> >  stream;

    yarp::sig::ImageOf<yarp::sig::PixelMono16>  imgDisp;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>     imgColor;

    double          focalLength;//!< camera focalLength
    double          baseLine;   //!< camera baseline
    double          dispScale;  //!< disparity scale factor
    yarp::sig::Matrix           T_CamInRobot;   //!< transformation matrix from robot to camera frame (left)

    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    attach(yarp::os::RpcServer &source);
    double  getPeriod();
    bool    updateModule();
public:
    bool compute3DCoor(yarp::sig::ImageOf<yarp::sig::PixelMono16> dispImg, const yarp::sig::Vector &pixel,
                       yarp::sig::Vector &point3D);
    bool compute3DCoorRect(yarp::sig::ImageOf<yarp::sig::PixelMono16> dispImg, const yarp::sig::Vector &tlPixel,
                           const yarp::sig::Vector &brPixel, const int &step,
                           yarp::sig::Vector &point3D);
//    template <class T>
////    yarp::sig::ImageOf<T> getBuffer(const rcg::Buffer *buffer, const int &_scale);
//    bool getBuffer(const rcg::Buffer *buffer, const int &_scale, yarp::sig::ImageOf<T> &yarpReturnImage);

    bool getBuffer8(const rcg::Buffer *buffer, const int &_scale,
                    yarp::sig::ImageOf<yarp::sig::PixelMono> &yarpReturnImage);

    bool getBuffer8andCvtColor(const rcg::Buffer *buffer, const int &_scale,
                               yarp::sig::ImageOf<yarp::sig::PixelRgb> &yarpReturnImage);

    bool getBuffer16(const rcg::Buffer *buffer, const int &_scale,
                     yarp::sig::ImageOf<yarp::sig::PixelMono16> &yarpReturnImage);
    /************************************************************************/
    // Thrift methods
    Point3D Rect(const int16_t tlx, const int16_t tly, const int16_t w, const int16_t h, const int16_t step)
    {
        yInfo("[%s] received : Rect %d %d %d %d %d", name.c_str(), tlx, tly, w, h, step);

        // Convert received pixel coordinate (in color image) to coordinate in disparity image
//        int16_t tlx_temp = int16_t(tlx*imgDisp.width()/imgColor.width());
//        int16_t tly_temp = int16_t(tly*imgDisp.height()/imgColor.height());

//        yInfo("[%s] converted: Rect %d %d %d %d %d", name.c_str(), tlx_temp, tly_temp, w, h, step);
        if (tlx>=0 && tly>=0 && w>=0 && h>=0)
        {
            yarp::sig::Vector tl(2,0.0), br(2,0.0), pt3D(3,0.0);
            tl[0] = tlx;
            tl[1] = tly;
            br[0] = tlx+w;
            br[1] = tly+h;
    //        Point3D _pt3D();
            if (imgDisp.width()>0 && imgDisp.height()>0)
                if (compute3DCoorRect(imgDisp,tl,br,step,pt3D))
                {
    //                _pt3D(). = pt3D[0];
    //                _pt3D().y = pt3D[1];
    //                _pt3D().z = pt3D[2];
                    yInfo("3D coordinator of [%s] pixel of disp image: %s",tl.toString(3,3).c_str(),
                        pt3D.toString(3,3).c_str());
                    return Point3D(pt3D[0],pt3D[1],pt3D[2]);
                }
            else
                return Point3D();
        }
        else
            return Point3D();

    }

};

#endif // RCYARPWRAPPER_H
