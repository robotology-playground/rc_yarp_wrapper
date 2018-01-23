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

class rcYarpWrapper : public yarp::os::RFModule
{
protected:
    double          period;
    std::string     name;       //!< module name
    int             scale;      //!< scale factor to resize original image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > port_mono, port_depth, port_conf;
    std::shared_ptr<rcg::Device>                dev;
    std::vector<std::shared_ptr<rcg::Stream> >  stream;

    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
//    bool    attach(RpcServer &source);
    double  getPeriod();
    bool    updateModule();
public:
    yarp::sig::ImageOf<yarp::sig::PixelMono> getBuffer(const rcg::Buffer *buffer, const int &_scale);
//    rcYarpWrapper();
};

#endif // RCYARPWRAPPER_H