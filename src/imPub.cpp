/*
Created by Jan Backhaus on 12.12.18 as part of the project ros_dji_link.
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <fstream>

#include <string>
#include <unistd.h>
#include <ros/node_handle.h>
#include "dji_link/GeoImageCompressed.h"
#include <std_msgs/String.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <exiv2/exiv2.hpp>



using namespace std;

void publishGeoImage(std::string& file);
void getInfos(const std_msgs::String::ConstPtr& msg);

sensor_msgs::NavSatFix 		exivStringToRosGPS( Exiv2::ExifData & aExifData );
std::vector<double> 		readXmpDataFromImage(Exiv2::XmpData & aXmpData );
double 						degMinSecToDecimal(double aArray[3]);

ros::Publisher image_pub;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcpimPub");
    ros::NodeHandle n;

    std::string geo_image_topic, filename_topic;

    n.getParam("geoImageTopic", geo_image_topic);
    n.getParam("djiFilenameTopic", filename_topic);

    image_pub = n.advertise<dji_link::GeoImageCompressed>(geo_image_topic,10);
    ros::Subscriber path_sub = n.subscribe(filename_topic, 10, getInfos);

    ros::Rate loop_rate(10);

    int rosspincounter = 0;
    while(ros::ok()){

        ros::spinOnce();
    	rosspincounter++;
    	loop_rate.sleep();
    }
 
}



void getInfos(const std_msgs::String::ConstPtr& msg)
{
    string filename = msg->data.c_str();
    ROS_INFO_STREAM("Received Filename: "+filename);
    if (filename.empty()){
        ROS_INFO_STREAM("No image found!\n");
    }else{
        publishGeoImage(filename);
    }
    return;
}


void publishGeoImage(std::string& filename)
{
    dji_link::GeoImageCompressed geoImage;
    cv::Mat im = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage cvImage;
    cvImage.image = im;
    cvImage.encoding = sensor_msgs::image_encodings::RGB8;

    geoImage.imagedata.format = "bgr8";
    geoImage.imagedata = *cvImage.toCompressedImageMsg();

    sensor_msgs::NavSatFix gpsdata;
    

    //Exiv2
    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(filename);
    if (image.get() != 0){
        image->readMetadata();
        Exiv2::ExifData &exifData = image->exifData();
        Exiv2::XmpData &xmpData = image->xmpData();
        if (exifData.empty()){
            ROS_INFO_STREAM("No ExifData found in the file!");
            return;
        }

        gpsdata = exivStringToRosGPS( exifData );
        std::vector <double> xmpdat = readXmpDataFromImage( xmpData );

        
        geoImage.gpsdata=gpsdata;
        geoImage.heading.data = xmpdat[1];
        geoImage.baroHeight.data = xmpdat[0];
        geoImage.header.stamp = ros::Time::now();

        geoImage.imagename.data = exifData["Exif.Image.ImageDescription"].toString();
        image_pub.publish(geoImage);
    }

 
   
}


sensor_msgs::NavSatFix exivStringToRosGPS(Exiv2::ExifData & aExifData )
{
    // TODO: Based on N or S, E or W and ASL or BSL calculate the values.
    sensor_msgs::NavSatFix gps_;
    // Altitude
    // if ( vect[1] = "0" ) vect[1] = "0";      // Above Sea Level
    // else 
    // vect[2] =  std::to_string((int) floor(fabs(gps_.altitude))) + "/1";   
    gps_.altitude = aExifData["Exif.GPSInfo.GPSAltitude"].toFloat();


    // Latitude
    double lLatitudeArr[3];
    //if ( gps_.latitude >= 0.0 ) vect[3] = "N";  // Above Equator
    //else vect[3] = "S";
    lLatitudeArr[0] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(0);
    lLatitudeArr[1] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(1);
    lLatitudeArr[2] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(2);

    gps_.latitude = degMinSecToDecimal(lLatitudeArr);    

    // Longitude
    double lLongitudeArr[3];
    //if ( gps_.longitude >= 0.0 ) vect[5] = "E";     // East of green meridian
    //else vect[5] = "W";
    lLongitudeArr[0] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(0);
    lLongitudeArr[1] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(1);
    lLongitudeArr[2] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(2);

    gps_.longitude = degMinSecToDecimal(lLongitudeArr);

    return gps_;
}

double degMinSecToDecimal(double aArray[3])
{
    double result;
    double angleDeg = aArray[0];
    double angleMin = aArray[1]/60;
    double angleSec = aArray[2]/3600;
    result = angleDeg + angleMin + angleSec;
    return result;
}

std::vector <double> readXmpDataFromImage(Exiv2::XmpData & aXmpData )
{
    std::vector <double> result(2);
    result[0] = aXmpData["Xmp.drone-dji.RelativeAltitude"].toFloat();
    result[1] = aXmpData["Xmp.drone-dji.FlightYawDegree"].toFloat();

    return result;
}