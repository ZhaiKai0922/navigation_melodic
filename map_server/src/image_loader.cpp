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

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>

#include "map_server/image_loader.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_server
{

void
loadMapDefault(nav_msgs::GetMap::Response* resp,std::string &fname)
{
  std::string pgmdir,yamldir,jsondir;
  getWallDir(fname,&pgmdir,&yamldir,&jsondir);
  MAPMETA myMapMeta = getMapMetaData(yamldir);
  
  std::vector<LINE> virtualWalls;


  cv::Mat image = cv::imread(pgmdir, cv::IMREAD_GRAYSCALE);
  // Copy the image data into the map structure
  resp->map.info.width = image.cols;
  resp->map.info.height = image.rows;

  resp->map.info.resolution = myMapMeta.res;
  resp->map.info.origin.position.x = myMapMeta.origin[0];
  resp->map.info.origin.position.y = myMapMeta.origin[1];
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(0, 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();
  
  int width = resp->map.info.width, height = resp->map.info.height;
  resp->map.data.resize(width * height);

  for (size_t i = 0; i < height; i++)
    {
      for (size_t j = 0; j < width; j++)
      {
        double occ = (255 - image.at<uchar>(height - 1 - i, j)) / 255.0;//opencv以图片左上角为原点,而occupangrid以图片左下角为原点,因此需要颠倒
        if (occ > myMapMeta.occ_th)
        {
          resp->map.data[i*width + j] = 100;
        }
        else if (occ < myMapMeta.free_th)
        {
          resp->map.data[i*width + j] = 0;
        }
        else
        {
          resp->map.data[i*width + j] = -1;
        }
      }
    }

  cv::Mat imagenav(height, width, CV_8UC1);
  ////获得基本地图，格式为cv::Mat
	for (int i = 0; i < height; i++)
  {
		for (int j = 0; j < width; j++)
		{
			if (resp->map.data[i * width + j] == 0)
			{
				imagenav.at<uchar>(i, j) = 254; //opencv以图片左上角为原点,因此这里opencv中的图片其实是颠倒的
			}
			else if (resp->map.data[i * width + j] == 100)
			{
				imagenav.at<uchar>(i, j) = 0;
			}
			else
			{
				imagenav.at<uchar>(i, j) = 205;
			}
		}
  }
  
  Json::Value walls;
  if(readJsonFile(jsondir.c_str(),&walls))
  {
    virtualWalls.resize(walls["line_list"].size());
    for (int i = 0; i < virtualWalls.size(); i++)
    {
      geometry_msgs::Point32 g0,g1;
      virtualWalls[i].id = walls["line_list"][i]["line_id"].asInt();
      virtualWalls[i].start.x = walls["line_list"][i]["start"]["x"].asDouble();
      virtualWalls[i].start.y = walls["line_list"][i]["start"]["y"].asDouble();
      virtualWalls[i].end.x = walls["line_list"][i]["end"]["x"].asDouble();
      virtualWalls[i].end.y = walls["line_list"][i]["end"]["y"].asDouble();
      g0.x = virtualWalls[i].start.x;
      g0.y = virtualWalls[i].start.y;
      g1.x = virtualWalls[i].end.x;
      g1.y = virtualWalls[i].end.y;
      if(isWorldPointInMap(g0,resp->map.info) && isWorldPointInMap(g1,resp->map.info))
      {
        cv::Point p0 = worldPointToCVPoint(g0,resp->map.info);
        cv::Point p1 = worldPointToCVPoint(g1,resp->map.info);
        cv::line(imagenav, p0, p1, cv::Scalar(0), 1);
      }
    }
  }
  else ROS_WARN("can't find virtualWalls");


  ////将cv::Mat转为occupancyGrid
  resp->map.data.resize(height * width);

  auto fun = [=](const cv::Mat & image,nav_msgs::OccupancyGrid & map)
	{
		for (int i = 0; i < height; i++)
			for (int j = 0; j < width; j++)
			{
				double occ = (255 - image.at<uchar>(i, j)) / 255.0; //opencv以图片左上角为原点,而occupangrid以图片左下角为原点.但由于在前面坐标系变换的时候没有进行颠倒,这里拷贝再进行一次颠倒,导致最后图片方向正常
				if (occ > myMapMeta.occ_th)
				{
					map.data[i * width + j] = 100;
				}
				else if (occ < myMapMeta.free_th)
				{
					map.data[i * width + j] = 0;
				}
				else
				{
					map.data[i * width + j] = -1;
				}
			}
	};
	
	fun(imagenav,resp->map);

}


void getWallDir(std::string &fname,std::string *pgmDir,std::string *yamlDir,std::string *jsonDir)
{
  int index_end = fname.rfind(".");
  int index_start = fname.rfind("/");
  std::string mapname = fname.substr((index_start + 1),(index_end - index_start -1));
  std::string root = getenv("HOME");
  *pgmDir = root + "/maps/" + mapname + ".pgm";
  *yamlDir = root + "/maps/" + mapname + ".yaml";
  *jsonDir = root + "/forbidden_line_data/" + mapname + ".json";
}

void publishNavMap(std::string &fname)
{
  
  std::string pgmdir,yamldir,jsondir;
  getWallDir(fname,&pgmdir,&yamldir,&jsondir);
  MAPMETA myMapMeta = getMapMetaData(yamldir);
  
  nav_msgs::OccupancyGrid mapnav;
  std::vector<LINE> virtualWalls;


  cv::Mat image = cv::imread(pgmdir, cv::IMREAD_GRAYSCALE);
  // Copy the image data into the map structure
  mapnav.info.width = image.cols;
  mapnav.info.height = image.rows;

  mapnav.info.resolution = myMapMeta.res;
  mapnav.info.origin.position.x = myMapMeta.origin[0];
  mapnav.info.origin.position.y = myMapMeta.origin[1];
  mapnav.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(0, 0, 0);
  mapnav.info.origin.orientation.x = q.x();
  mapnav.info.origin.orientation.y = q.y();
  mapnav.info.origin.orientation.z = q.z();
  mapnav.info.origin.orientation.w = q.w();
  
  int width = mapnav.info.width, height = mapnav.info.height;
  mapnav.data.resize(width * height);

  for (size_t i = 0; i < height; i++)
    {
      for (size_t j = 0; j < width; j++)
      {
        double occ = (255 - image.at<uchar>(height - 1 - i, j)) / 255.0;//opencv以图片左上角为原点,而occupangrid以图片左下角为原点,因此需要颠倒
        if (occ > myMapMeta.occ_th)
        {
          mapnav.data[i*width + j] = 100;
        }
        else if (occ < myMapMeta.free_th)
        {
          mapnav.data[i*width + j] = 0;
        }
        else
        {
          mapnav.data[i*width + j] = -1;
        }
      }
    }

  cv::Mat imagenav(height, width, CV_8UC1);
  ////获得基本地图，格式为cv::Mat
	for (int i = 0; i < height; i++)
  {
		for (int j = 0; j < width; j++)
		{
			if (mapnav.data[i * width + j] == 0)
			{
				imagenav.at<uchar>(i, j) = 254; //opencv以图片左上角为原点,因此这里opencv中的图片其实是颠倒的
			}
			else if (mapnav.data[i * width + j] == 100)
			{
				imagenav.at<uchar>(i, j) = 0;
			}
			else
			{
				imagenav.at<uchar>(i, j) = 205;
			}
		}
  }
  
  Json::Value walls;
  if(readJsonFile(jsondir.c_str(),&walls))
  {
    virtualWalls.resize(walls["line_list"].size());
    for (int i = 0; i < virtualWalls.size(); i++)
    {
      geometry_msgs::Point32 g0,g1;
      virtualWalls[i].id = walls["line_list"][i]["line_id"].asInt();
      virtualWalls[i].start.x = walls["line_list"][i]["start"]["x"].asDouble();
      virtualWalls[i].start.y = walls["line_list"][i]["start"]["y"].asDouble();
      virtualWalls[i].end.x = walls["line_list"][i]["end"]["x"].asDouble();
      virtualWalls[i].end.y = walls["line_list"][i]["end"]["y"].asDouble();
      g0.x = virtualWalls[i].start.x;
      g0.y = virtualWalls[i].start.y;
      g1.x = virtualWalls[i].end.x;
      g1.y = virtualWalls[i].end.y;
      if(isWorldPointInMap(g0,mapnav.info) && isWorldPointInMap(g1,mapnav.info))
      {
        cv::Point p0 = worldPointToCVPoint(g0,mapnav.info);
        cv::Point p1 = worldPointToCVPoint(g1,mapnav.info);
        cv::line(imagenav, p0, p1, cv::Scalar(0), 1);
      }
    }
  }
  else ROS_WARN("can't find virtualWalls");


  ////将cv::Mat转为occupancyGrid
  mapnav.data.resize(height * width);

  auto fun = [=](const cv::Mat & image,nav_msgs::OccupancyGrid & map)
	{
		for (int i = 0; i < height; i++)
			for (int j = 0; j < width; j++)
			{
				double occ = (255 - image.at<uchar>(i, j)) / 255.0; //opencv以图片左上角为原点,而occupangrid以图片左下角为原点.但由于在前面坐标系变换的时候没有进行颠倒,这里拷贝再进行一次颠倒,导致最后图片方向正常
				if (occ > myMapMeta.occ_th)
				{
					map.data[i * width + j] = 100;
				}
				else if (occ < myMapMeta.free_th)
				{
					map.data[i * width + j] = 0;
				}
				else
				{
					map.data[i * width + j] = -1;
				}
			}
	};
	
	fun(imagenav,mapnav);

  mapnav.info.map_load_time = ros::Time::now();
  mapnav.header.frame_id = "map";
  mapnav.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               mapnav.info.width,
               mapnav.info.height,
               mapnav.info.resolution);

  mapnav_pub.publish(mapnav);

  
}

MAPMETA getMapMetaData(std::string yamldir)
{
  MAPMETA mapmetadata;
  YAML::Node conf = YAML::LoadFile(yamldir);
  mapmetadata.res = conf["resolution"].as<double>();
  mapmetadata.negate = conf["negate"].as<int>();
  mapmetadata.negate = bool(mapmetadata.negate);
  mapmetadata.occ_th = conf["occupied_thresh"].as<double>();
  mapmetadata.free_th = conf["free_thresh"].as<double>();
  mapmetadata.origin[0] = conf["origin"][0].as<double>();
  mapmetadata.origin[1] = conf["origin"][1].as<double>();
  mapmetadata.origin[2] = conf["origin"][2].as<double>();
  mapmetadata.pgm_name = conf["image"].as<std::string>();
  return mapmetadata;
}


bool readJsonFile(const char *jsonFileName,Json::Value *jsonInfo)
{
  std::ifstream jsonFile(jsonFileName,std::ios::binary);
    if(!jsonFile.is_open())
    {
        ROS_WARN("Map_server load forbidden_line Json File OPEN %s ERROR\n",jsonFileName);
        return false;
    }
    else
    {
        Json::Reader reader;
        if(reader.parse(jsonFile,*jsonInfo))
        {
            jsonFile.close();
            return true;
        }
        else
        {
            ROS_WARN("Map_server read forbidden_line JsonFile parse json ERROR\n");
            jsonFile.close();
            return false;
        }
    }
    return true;
}


}
