/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MAP_SERVER_MAP_SERVER_H
#define MAP_SERVER_MAP_SERVER_H

/*
 * Author: Brian Gerkey
 */
#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>

#include "yaml-cpp/yaml.h"
#include <jsoncpp/json/json.h>

/** Map mode
 *  Default: TRINARY -
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - Unknown
 *  SCALE -
 *      alpha < 1.0 - Unknown
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - f( (free_th, occ_th) ) = (0, 100)
 *          (linearly map in between values to (0,100)
 *  RAW -
 *      value = value
 */
enum MapMode {TRINARY, SCALE, RAW};


extern ros::Publisher mapnav_pub;

namespace map_server
{
struct MAPMETA
{
    double res;
    bool negate;
    double occ_th;
    double free_th;
    double origin[3];
    std::string pgm_name; 
};

struct POINT
{
    double x;
    double y;
};

struct LINE
{
    int id;
    POINT start;
    POINT end;
};

/** Read the image from file and fill out the resp object, for later
 * use when our services are requested.
 *
 * @param resp The map wil be written into here
 * @param fname The image file to read from
 * @param res The resolution of the map (gets stored in resp)
 * @param negate If true, then whiter pixels are occupied, and blacker
 *               pixels are free
 * @param occ_th Threshold above which pixels are occupied
 * @param free_th Threshold below which pixels are free
 * @param origin Triple specifying 2-D pose of lower-left corner of image
 * @param mode Map mode
 * @throws std::runtime_error If the image file can't be loaded
 * */
void loadMapDefault(nav_msgs::GetMap::Response* resp,std::string &fname);
void publishNavMap(std::string &fname);


bool readJsonFile(const char *jsonFileName,Json::Value *jsonInfo);

void getWallDir(std::string &fname,std::string *pgmDir,std::string *yamlDir,std::string *jsonDir);
MAPMETA getMapMetaData(std::string yamldir);


cv::Point worldPointToCVPoint(const geometry_msgs::Point32 &point, const nav_msgs::MapMetaData &mapInfo)
{
    auto x = (point.x - mapInfo.origin.position.x) / mapInfo.resolution;
    auto y = (point.y - mapInfo.origin.position.y) / mapInfo.resolution;
    return {static_cast<int>(x), static_cast<int>(y)};
}
bool isWorldPointInMap(geometry_msgs::Point32 &point, const nav_msgs::MapMetaData &mapInfo)
{
    int mapX = static_cast<int>((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
    int mapY = static_cast<int>((point.y - mapInfo.origin.position.y) / mapInfo.resolution);

    bool beyondMap = (mapX<0 || mapX>mapInfo.width || mapY<0 || mapY>mapInfo.height);
    return !beyondMap;
}



}



#endif
