# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: NGUYEN Dong Hai Phuong 
# Emails: phuong.nguyen@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# rcYarpWrapper.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct Point3D {
  1: double x;
  2: double y;
  3: double z;
}

/**
* skeleton3D_IDL
*
* IDL Interface to \ref skeleton3D services.
*/
service rcYarpWrapper_IDL
{
  /**
  * Sets body valence (threat).
  * @param _valence thread value for all body parts.
  * @return true/false on success/failure.
  */
  Point3D Rect(1:i16 tlx, 2:i16 tly, 3:i16 w, 4:i16 h, 5:i16 step);

  

  /**
  * Enables the function to create a fake rightHand
  * @return true/false on success/failure.
  */
  //bool enable_fake_hand();

  /**
  * Disables the function to create a fake rightHand
  * @return true/false on success/failure.
  */
  //bool disable_fake_hand();
}
