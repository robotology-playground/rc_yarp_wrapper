#include "rcYarpWrapper.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
//using namespace iCub::ctrl;
//rcYarpWrapper::rcYarpWrapper()
//{

//}
ImageOf<PixelMono> rcYarpWrapper::getBuffer(const rcg::Buffer *buffer, const int &_scale)
{

    ImageOf<PixelMono> yarpReturnImage;
    // prepare file name

    double t=buffer->getTimestampNS()/1000000000.0;

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

        yarpReturnImage.wrapIplImage(image2);

    }
    else if (!buffer->getImagePresent())
    {
        std::cerr << "getBuffer(): Received buffer without image" << std::endl;
        return yarpReturnImage;
    }
    else if (buffer->getIsIncomplete())
    {
        std::cerr << "getBuffer(): Received buffer without image" << std::endl;
        return yarpReturnImage;
    }

    return yarpReturnImage;
}

bool    rcYarpWrapper::configure(ResourceFinder &rf)
{
    name = rf.check("name",Value("rc_img_conveyor")).asString().c_str();
    period = rf.check("period",Value(0.0)).asDouble();    // as default, update module as soon as possible

    bool getMono = rf.check("mono",Value(1)).asBool();
    bool getCombined = rf.check("combined",Value(0)).asBool();
    bool getDepth = rf.check("depth",Value(1)).asBool();
    bool getError = rf.check("error",Value(0)).asBool();
    bool getConfidence = rf.check("confidence",Value(0)).asBool();

    scale = rf.check("scale",Value(4)).asInt();
    yInfo("scale %d",scale);

    string device = rf.check("device",Value("eno1:00_14_2d_2c_6e_56")).asString().c_str();  // right one should be get by running rcdiscover-gui
    // find specific device accross all systems and interfaces and open it
    dev=rcg::getDevice(device.c_str());
    stream.clear();
    if (dev)
    {
        yInfo("[%s] Device %s available",name.c_str(),device.c_str());
        dev->open(rcg::Device::CONTROL);
        std::shared_ptr<GenApi::CNodeMapRef> nodemap=dev->getRemoteNodeMap();

        rcg::setEnum(nodemap, "ComponentSelector", "Disparity", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getDepth, true);
        rcg::setEnum(nodemap, "ComponentSelector", "Intensity", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getMono, true);
        rcg::setEnum(nodemap, "ComponentSelector", "IntensityCombined", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getCombined, true);
        rcg::setEnum(nodemap, "ComponentSelector", "Error", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getError, true);
        rcg::setEnum(nodemap, "ComponentSelector", "Confidence", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getConfidence, true);

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

        port_mono.open(("/"+name+"/mono").c_str());
        port_depth.open(("/"+name+"/depth").c_str());
        port_conf.open(("/"+name+"/confidence").c_str());

        // open stream and get images
        stream=dev->getStreams();

        if (stream.size()>0)
        {
            stream[0]->open();
            stream[0]->startStreaming();
            yDebug("[%s] Start streaming", name.c_str());
        }
        else
        {
            yError("[%s] No stream available",name.c_str());
            return false;
        }

    }
    else
    {
        yError("Device %s not found",device.c_str());
        return false;
    }

    return true;
}

bool    rcYarpWrapper::interruptModule()
{
    yDebug("[%s] Interupt module",name.c_str());
//    rpcPort.interrupt();
    if (stream.size()>0)
    {
        stream[0]->stopStreaming();
        stream[0]->close();
    }
    port_depth.interrupt();
    port_mono.interrupt();
    port_conf.interrupt();
    return true;
}

bool    rcYarpWrapper::close()
{
    yDebug("[%s] closing module",name.c_str());
//    rpcPort.close();
    port_depth.close();
    port_mono.close();
    port_conf.close();
    dev->close();
    rcg::System::clearSystems();

    return true;
}

double  rcYarpWrapper::getPeriod()
{
    return period;
}

bool    rcYarpWrapper::updateModule()
{
    // grab next image with timeout of 30 mili-seconds

    int retry=5;
    while (retry > 0)
    {
        const rcg::Buffer *buffer=stream[0]->grab(30);

        if (buffer != 0)
        {
            ImageOf<PixelMono> img = getBuffer(buffer, scale);
            if (img.getRowSize()>0)
            {
                uint64_t format=buffer->getPixelFormat();
                switch (format)
                {
                case Mono8: // store 8 bit monochrome image
                case Confidence8:
                case Error8:
                {
                    if (format == Mono8)
                    {
                        //TODO: export image to monochrome port
                        port_mono.prepare() = img;
                        port_mono.write();
                    }
                    else if (format == Confidence8)
                    {
                        port_conf.prepare() = img;
                        port_conf.write();
                    }
                    else if (format == Error8)
                    {

                    }
                }
                    break;

                case Coord3D_C16: // store 16 bit monochrome image
                {

                    // copy image data, pgm is always big endian
                    //TODO: export image to disparity port
                    port_depth.prepare() = img;
                    port_depth.write();
                    if (buffer->isBigEndian())
                    {

                    }
                    else
                    {

                    }

                }
                    break;
                }
            }

        }
        else
        {
            yError() << "Cannot grab images";
            break;
        }

        retry--;
    }

    return true;
}
