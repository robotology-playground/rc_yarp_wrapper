/*
 * This file is part of the rc_genicam_api package.
 *
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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

#ifdef WIN32
#undef min
#undef max
#endif

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

namespace
{

/**
  Store image given in buffer in PGM or PPM format.
*/

//std::string storeBuffer(const rcg::Buffer *buffer)
ImageOf<PixelMono> storeBuffer(const rcg::Buffer *buffer, const int &_scale)
{

    ImageOf<PixelMono> yarpReturnImage;
    // prepare file name

    std::ostringstream name;

    double t=buffer->getTimestampNS()/1000000000.0;

    name << "image_" << std::setprecision(16) << t;

    // store image (see e.g. the sv tool of cvkit for show images)

    if (!buffer->getIsIncomplete() && buffer->getImagePresent())
    {
        size_t width=buffer->getWidth();
        size_t height=buffer->getHeight();
        const unsigned char *p=static_cast<const unsigned char *>(buffer->getBase())+buffer->getImageOffset();
        cv::Mat image(height,width, CV_8U);
        cv::Mat dst;
        int scale = _scale;

        image.data = (unsigned char*) p;
        yInfo() <<"width "<< image.cols;
        yInfo() <<"height "<< image.rows;
        cv::resize(image, dst, cv::Size(int(height/scale),int(width/scale)));

        IplImage* image2;
        image2 = cvCreateImage(cvSize(int(height/scale),int(width/scale)),8,1);
        IplImage ipltemp=dst;
        cvCopy(&ipltemp,image2);

//        IplImage copy = dst;
//        IplImage* image2 = &copy;


//        ImageOf<PixelMono> yarpReturnImage;
        yarpReturnImage.wrapIplImage(image2);
//        yarp::sig::file::write(yarpReturnImage,"test.pgm");

        size_t px=buffer->getXPadding();

        uint64_t format=buffer->getPixelFormat();
        switch (format)
        {
        case Mono8: // store 8 bit monochrome image
        case Confidence8:
        case Error8:
        {
            if (format == Mono8)
            {
                name << "_mono.pgm";
                //TODO: export image to monochrome port
            }
            else if (format == Confidence8)
            {
                name << "_conf.pgm";
            }
            else if (format == Error8)
            {
                name << "_err.pgm";
            }
        }
            break;

        case Coord3D_C16: // store 16 bit monochrome image
        {
            name << "_disp.pgm";
            std::ofstream out(name.str(), std::ios::binary);

            out << "P5" << std::endl;
            out << width << " " << height << std::endl;
            out << 65535 << "\n";

            std::streambuf *sb=out.rdbuf();

            // copy image data, pgm is always big endian
            //TODO: export image to disparity port

            if (buffer->isBigEndian())
            {

            }
            else
            {

            }

            out.close();
        }
            break;

        case YCbCr411_8: // convert and store as color image
        {
            name << "_color.ppm";
            std::ofstream out(name.str(), std::ios::binary);

            out << "P6" << std::endl;
            out << width << " " << height << std::endl;
            out << 255 << "\n";

            std::streambuf *sb=out.rdbuf();

            size_t pstep=(width>>2)*6+px;
            for (size_t k=0; k<height && out.good(); k++)
            {
                for (size_t i=0; i<width; i+=4)
                {
                    uint8_t rgb[12];
                    rcg::convYCbCr411toQuadRGB(rgb, p, static_cast<int>(i));

                    for (int j=0; j<12; j++)
                    {
                        sb->sputc(static_cast<char>(rgb[j]));
                    }
                }

                p+=pstep;
            }

            out.close();
        }
            break;

        default:
            std::cerr << "storeBuffer(): Unknown pixel format: "
                      << GetPixelFormatName(static_cast<PfncFormat>(buffer->getPixelFormat()))
                      << std::endl;
//            return std::string();
            return yarpReturnImage;
            break;
        }
    }
    else if (!buffer->getImagePresent())
    {
        std::cerr << "storeBuffer(): Received buffer without image" << std::endl;
//        return std::string();
        return yarpReturnImage;
    }
    else if (buffer->getIsIncomplete())
    {
        std::cerr << "storeBuffer(): Received buffer without image" << std::endl;
//        return std::string();
        return yarpReturnImage;
    }

//    return name.str();
    return yarpReturnImage;
}

}

int main(int argc, char *argv[])
{
    // Initialize network
    Network yarp;
    // Make a port for reading and writing images
    BufferedPort<ImageOf<PixelMono> > port_mono, port_depth;
    // Get command line options
    Property options;
    options.fromCommand(argc,argv);
    // Set the name of the port (use "/worker" if there is no --name option)
    ConstString portName = options.check("name",Value("/worker")).asString();
    std::string name = options.check("name",Value("rc_img_conveyor")).asString().c_str();
    port_mono.open(("/"+name+"/mono").c_str());
    port_depth.open(("/"+name+"/depth").c_str());

    try
    {
        if (argc >= 2)
        {
            // find specific device accross all systems and interfaces and open it

            std::shared_ptr<rcg::Device> _dev=rcg::getDevice(argv[1]);

            if (_dev)
            {
                _dev->open(rcg::Device::CONTROL);
                std::shared_ptr<GenApi::CNodeMapRef> nodemap=_dev->getRemoteNodeMap();

                // set values as given on the command line

                int n=1;

                int i=2;
                int scale =2;
                while (i < argc)
                {
                    // split argument in key and value

                    std::string key=argv[i++];
                    std::string value;

                    size_t k=key.find('=');
                    if (k != std::string::npos)
                    {
                        value=key.substr(k+1);
                        key=key.substr(0, k);
                    }

                    if (key == "n") // set number of images
                    {
                        n=std::max(1, std::stoi(value));
                    }
                    else // set key=value pair through GenICam
                    {
                        rcg::setString(nodemap, key.c_str(), value.c_str(), true);
                    }

                    if (key == "scale")
                    {
                        scale = std::stoi(value);
                    }
                    yInfo("scale %d",scale);
                }

                // print enabled streams

                {
                    std::vector<std::string> component;

                    rcg::getEnum(nodemap, "ComponentSelector", component, false);

                    if (component.size() > 0)
                    {
                        std::cout << std::endl;
                        std::cout << "Available components (1 means enabled, 0 means disabled):" << std::endl;
                        std::cout << std::endl;

                        for (size_t k=0; k<component.size(); k++)
                        {
                            rcg::setEnum(nodemap, "ComponentSelector", component[k].c_str(), true);

                            std::cout << component[k] << ": ";
                            std::cout << rcg::getBoolean(nodemap, "ComponentEnable", true, true);
                            std::cout << std::endl;
                        }

                        std::cout << std::endl;
                    }
                }

                // open stream and get n images

                std::vector<std::shared_ptr<rcg::Stream> > stream=_dev->getStreams();

                if (stream.size() > 0)
                {
                    yInfo("stream size: %lu",stream.size());
                    // opening first stream

                    stream[0]->open();
                    stream[0]->startStreaming();

                    for (int k=0; k<n; k++)
                    {
                        // grab next image with timeout of 30 mili-seconds

                        int retry=5;
                        while (retry > 0)
                        {
                            const rcg::Buffer *buffer=stream[0]->grab(30);

                            if (buffer != 0)
                            {
//                                std::string name=storeBuffer(buffer);
                                ImageOf<PixelMono> img = storeBuffer(buffer, scale);
                                if (img.getRowSize()>0)
                                {
                                    yDebug("img row %d",img.getRowSize());
                                    if (img.getRowSize()==int(960/scale))
                                    {
                                        yDebug("mono image");

                                        port_mono.prepare() = img;
                                        port_mono.write();
                                    }
                                    else if (img.getRowSize()==int(480/scale))
                                    {
                                        yDebug("depth image");

                                        port_depth.prepare() = img;
                                        port_depth.write();
                                    }

                                }

                            }
                            else
                            {
                                std::cerr << "Cannot grab images" << std::endl;
                                break;
                            }

                            retry--;
                        }
                    }

                    stream[0]->stopStreaming();
                    stream[0]->close();
                }
                else
                {
                    std::cerr << "No streams available" << std::endl;
                }

                _dev->close();
            }
            else
            {
                std::cerr << "Device '" << argv[1] << "' not found!" << std::endl;
            }
        }
        else
        {
            // show help

            std::cout << argv[0] << " [interface-id>:]<device-id> [n=<n>] [<key>=<value>] ..." << std::endl;
            std::cout << std::endl;
            std::cout << "Stores n images from the specified device after applying the given values." << std::endl;
            std::cout << "Components can be enabled with 'ComponentSelector=<component> ComponentEnable=1'." << std::endl;
            std::cout << std::endl;
            std::cout << "<device-id>   Device from which data will be streamed" << std::endl;
            std::cout << "n=<n>         Number of images to receive. Default is 1" << std::endl;
            std::cout << "<key>=<value> Values set via GenICam before streaming images" << std::endl;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }

    rcg::System::clearSystems();

    return 0;
}
