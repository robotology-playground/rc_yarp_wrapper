#include "rcYarpWrapper.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
//using namespace iCub::ctrl;
//rcYarpWrapper::rcYarpWrapper()
//{

//}
bool rcYarpWrapper::compute3DCoorRect(yarp::sig::ImageOf<PixelMono16> dispImg, const Vector &tlPixel,
                                      const Vector &brPixel, const int &step,
                                      Vector &point3D)
{
    int cnt=0;
    for (int u=tlPixel[0]; u<brPixel[0]; u+=step)   //row
    {
        for (int v=tlPixel[1]; v<brPixel[1]; v+=step)     //column
        {
            Vector pointTemp(3,0.0);
            Vector pixel(2,0.0);
            pixel[0] = u;   pixel[1] = v;
            if (compute3DCoor(dispImg,pixel,pointTemp))
            {
                point3D+=pointTemp;
                cnt++;
            }
        }
    }
    if (cnt>0)
    {
        point3D/=cnt;
        yInfo("point3D in cam frame  : %s", point3D.toString(3,3).c_str());
        Vector point3D_temp(4,1.0);
        point3D_temp.setSubvector(0,point3D);
        point3D_temp = T_CamInRobot * point3D_temp;
        point3D = point3D_temp.subVector(0,2);
        yInfo("point3D in robot frame: %s", point3D.toString(3,3).c_str());
        return true;
    }
    else
        return false;
}

bool rcYarpWrapper::compute3DCoor(yarp::sig::ImageOf<PixelMono16> dispImg, const Vector &pixel,
                                  Vector &point3D)
{
    if (pixel.size()==2)
    {
        double imgW = double(dispImg.width());
        double imgH = double(dispImg.height());
        point3D.resize(3);
        // TODO if(dispImg.isPixel(pixel[0], pixel[1])
        int disp=0;
        if(dispImg.isPixel(pixel[0], pixel[1]))
            disp = int(dispImg.pixel(pixel[0],pixel[1]));

        if (disp!=0)
        {
            double d = double(disp)*dispScale;
            point3D[0] = (pixel[0]-imgW/2)*baseLine/d;
            point3D[1] = (pixel[1]-imgH/2)*baseLine/d;
            point3D[2] = focalLength*imgW*baseLine/d;           

//            yInfo("disparity at pixel [%s]: %d",pixel.toString(3,3).c_str(), disp);
//            yInfo("[%s] img width, height: %f, %f",name.c_str(),imgW, imgH);
//            yInfo("3D coordinator of pixel [%s]: %s",pixel.toString(3,3).c_str(),
//                  point3D.toString(3,3).c_str());

            return true;
        }
        else
        {
//            yWarning("[%s] disp==0, invalid disparity at the pixel [%f, %f]",name.c_str(), pixel[0], pixel[1]);
            return false;
        }
    }
    else
        return false;
}


bool rcYarpWrapper::getBuffer8(const rcg::Buffer *buffer, const int &_scale, yarp::sig::ImageOf<PixelMono> &yarpReturnImage)
{
    // prepare file name

    double t=buffer->getTimestampNS()/1000000000.0;

    if (!buffer->getIsIncomplete() && buffer->getImagePresent())
    {
        int mScale = _scale;
        size_t width=buffer->getWidth();
        size_t height=buffer->getHeight();
        const unsigned char *p=static_cast<const unsigned char *>(buffer->getBase())+buffer->getImageOffset();
        int imgType, imgDepth;
        imgType = CV_8U;
        imgDepth = 8;

        cv::Mat image(height,width,imgType);
        cv::Mat dst;

        image.data = (unsigned char*) p;

        cv::resize(image, dst, cvSize(int(width/mScale),int(height/mScale)));
//        cv::resize(image, dst, cv::Size(int(height/mScale),int(width/mScale)));

        IplImage* image2;
        image2 = cvCreateImage(cv::Size(int(width/mScale),int(height/mScale)),imgDepth,1);
//        image2 = cvCreateImage(cv::Size(int(height/mScale),int(width/mScale)),imgDepth,1);
        IplImage ipltemp=dst;
        cvCopy(&ipltemp,image2);

        yarpReturnImage.wrapIplImage(image2);

    }
    else if (!buffer->getImagePresent())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    else if (buffer->getIsIncomplete())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    return true;
}

bool rcYarpWrapper::getBuffer8andCvtColor(const rcg::Buffer *buffer, const int &_scale, yarp::sig::ImageOf<PixelRgb> &yarpReturnImage)
{
    // prepare file name

    double t=buffer->getTimestampNS()/1000000000.0;

    if (!buffer->getIsIncomplete() && buffer->getImagePresent())
    {
        int mScale = _scale;
        size_t width=buffer->getWidth();
        size_t height=buffer->getHeight();
        const unsigned char *p=static_cast<const unsigned char *>(buffer->getBase())+buffer->getImageOffset();
        int imgType, imgDepth;
        imgType = CV_8U;
        imgDepth = 8;

        cv::Mat image(height,width,imgType);
        cv::Mat dst;

        image.data = (unsigned char*) p;

        cv::resize(image, dst, cvSize(int(width/mScale),int(height/mScale)));

        // Convert to fake color
        cv::Mat color_image;
        cv::cvtColor(dst,color_image,CV_GRAY2RGB);

        // Mat to IplImage
        IplImage* image2;
        image2 = cvCreateImage(cv::Size(int(width/mScale),int(height/mScale)),imgDepth,3);
        IplImage ipltemp=color_image;
        cvCopy(&ipltemp,image2);


        yarpReturnImage.wrapIplImage(image2);

    }
    else if (!buffer->getImagePresent())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    else if (buffer->getIsIncomplete())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    return true;
}

bool rcYarpWrapper::getBuffer16(const rcg::Buffer *buffer, const int &_scale, yarp::sig::ImageOf<PixelMono16> &yarpReturnImage)
{
    // prepare file name
    double t=buffer->getTimestampNS()/1000000000.0;

    if (!buffer->getIsIncomplete() && buffer->getImagePresent())
    {
        int mScale = _scale;
        size_t width=buffer->getWidth();
        size_t height=buffer->getHeight();
        const unsigned char *p=static_cast<const unsigned char *>(buffer->getBase())+buffer->getImageOffset();
        int imgType, imgDepth;
        imgType = CV_16U;
        imgDepth = 16;

        cv::Mat image(height,width,imgType);
        cv::Mat dst;

        image.data = (unsigned char*) p;

        cv::resize(image, dst, cvSize(int(width/mScale),int(height/mScale)));

        IplImage* image2;
        image2 = cvCreateImage(cvSize(int(width/mScale),int(height/mScale)),imgDepth,1);
        IplImage ipltemp=dst;
        cvCopy(&ipltemp,image2);

        yarpReturnImage.wrapIplImage(image2);
        // TODO
        if (buffer->isBigEndian())
        {
        }
        else
        {
        }

    }
    else if (!buffer->getImagePresent())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    else if (buffer->getIsIncomplete())
    {
        yError() << "getBuffer(): Received buffer without image";
        return false;
    }
    return true;
}


bool    rcYarpWrapper::configure(ResourceFinder &rf)
{
    name = rf.check("name",Value("rc_img_conveyor")).asString().c_str();
    period = rf.check("period",Value(0.0)).asDouble();    // as default, update module as soon as possible

    bool getMono = rf.check("mono",Value(1)).asBool();
    bool getCombined = rf.check("combined",Value(0)).asBool();
    bool getDisp = rf.check("disp",Value(1)).asBool();
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

        // Set stream outputs
        rcg::setEnum(nodemap, "ComponentSelector", "Disparity", true);
        rcg::setBoolean(nodemap, "ComponentEnable", getDisp, true);
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

        // get focal length, baseline and disparity scale factor

        focalLength=rcg::getFloat(nodemap, "FocalLengthFactor", 0, 0, false);
        baseLine=rcg::getFloat(nodemap, "Baseline", 0, 0, true);
        dispScale=rcg::getFloat(nodemap, "Scan3dCoordinateScale", 0, 0, true);

        yInfo("focalLength: %f",focalLength);
        yInfo("baseline: %f",baseLine);
        yInfo("dispScale: %f",dispScale);

        port_mono.open(("/"+name+"/mono").c_str());
        port_disp.open(("/"+name+"/disp").c_str());
        port_conf.open(("/"+name+"/confidence").c_str());

        port_color.open(("/"+name+"/color").c_str());

        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

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

    Vector xyzrpy(6,0.0);
    // Calibrators
    if (rf.check("cam_in_robot"))
    {
        Bottle *camInRobotTransform = rf.find("cam_in_robot").asList();
        if ((!camInRobotTransform->isNull()) && (camInRobotTransform->size()==6))
        {
            for (int i=0; i<6; i++)
                xyzrpy[i] = camInRobotTransform->get(i).asDouble();
        }
        else
            yWarning("[%s] Found %s  in the config file but it is empty; using default", name.c_str(), "calibrator");

    }
    else
    {
         yWarning("[%s] Could not find %s  in the config file; using default", name.c_str(), "calibrator");
    }

//    Matrix T(4,4);
//    Vector rpy(3,0.0);
//    rpy[0] = -98.21*M_PI/180.0;
//    rpy[1] =   2.89*M_PI/180.0;
//    rpy[2] = -56.32*M_PI/180.0;
//    T(0,3)=-0.6008;
//    T(1,3)=-1.0761;
//    T(2,3)= 0.2629;
    Vector rpy = xyzrpy.subVector(3,5)*M_PI/180.0;
    T_CamInRobot = rpy2dcm(rpy);
    T_CamInRobot.setSubcol(xyzrpy.subVector(0,2),0,3);

    yDebug("T_CamInRobot = %s", T_CamInRobot.toString(3,3).c_str());

    return true;
}

bool    rcYarpWrapper::interruptModule()
{
    yDebug("[%s] Interupt module",name.c_str());
//    rpcPort.interrupt();
    port_disp.interrupt();
    port_mono.interrupt();
    port_conf.interrupt();
    return true;
}

bool    rcYarpWrapper::close()
{
    yDebug("[%s] closing module",name.c_str());
//    rpcPort.close();
    port_disp.close();
    port_mono.close();
    port_conf.close();
    if (dev && stream.size()>0)
    {
        stream[0]->stopStreaming();
        stream[0]->close();

    }
    dev->close();
    rcg::System::clearSystems();

    return true;
}

bool    rcYarpWrapper::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
//    return true;
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

        if (buffer !=0)
        {

                uint64_t format=buffer->getPixelFormat();
                switch (format)
                {
                    case Mono8:
                    {
//                        ImageOf<PixelRgb> img;
                        if (getBuffer8andCvtColor(buffer, scale, imgColor)) // color image has size of 240x320
                        {
                            port_color.prepare() = imgColor;
                            port_color.write();
                        }
                    }
                    case Confidence8:
                    case Error8:
                    {
                        ImageOf<PixelMono> img;
                        if(getBuffer8(buffer, scale,img))   // mono image has size of 240x320
                        {
//                            ImageOf<PixelMono> img= getBuffer8(buffer, scale);
//                            if (format == Mono8)
//                            {
//                                //TODO: export image to monochrome port
//                                port_mono.prepare() = img;
//                                port_mono.write();
//                            }
                            if (format == Confidence8)
                            {
                                port_conf.prepare() = img;
                                port_conf.write();
                            }
                            else if (format == Error8)
                            {

                            }
                        }
                    }
                        break;

                    case Coord3D_C16: // store 16 bit monochrome image
                    {
//                        ImageOf<PixelMono16> img;
                        if (getBuffer16(buffer, 1, imgDisp))        // Disparity map has size of 480x640
                        {
    //                        ImageOf<PixelMono16> img= getBuffer16(buffer, 1);
                            // copy image data, pgm is always big endian
                            //TODO: export image to disparity port
                            port_disp.prepare() = imgDisp;
                            port_disp.write();
                            Vector pixel(2,0.0), p_tl(2,0.0),p_br(2,0.0), pt3D(3,0.0);
                            pixel[0] = int(imgDisp.width()/2.0); //col
                            pixel[1] = int(imgDisp.height()/2.0); //row
//                            pixel[1] = 200;
//                            compute3DCoor(imgDisp,pixel, pt3D);

                            p_tl = pixel-1;
                            p_br = pixel+1;
                            if (compute3DCoorRect(imgDisp,p_tl,p_br,1,pt3D))
                                yInfo("3D coordinator of [%s] pixel of disp image: %s",pixel.toString(3,3).c_str(),
                                    pt3D.toString(3,3).c_str());
                        }

                    }
                    break;
                }
//            }

        }
        else
        {
//            yError() << "Cannot grab images"; //Deactivate for now
            break;
        }

        retry--;
    }

    return true;
}
