/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

namespace costmap_2d
{

void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  if (should_delete) old_h.deleteParam(name);
}

Costmap2DROS::Costmap2DROS(const std::string& name, tf2_ros::Buffer& tf) :
    layered_costmap_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    plugin_loader_("costmap_2d", "costmap_2d::Layer"),
    publisher_(NULL),
    dsrv_(NULL),
    footprint_padding_(0.0)
{
  // Initialize old pose with something
  //初始化 old_pose_
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle g_nh;

  // get global and robot base frame names
  // 2） 读取frame参数global_frame和robot_base_frame
  private_nh.param("global_frame", global_frame_, std::string("map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  // we need to make sure that the transform between the robot base frame and the global frame is available
   //确保机器人底盘坐标系和世界坐标系之间的有效转换
   //否则一直停留在这里阻塞
  while (ros::ok()
      && !tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
  {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
      ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  ////检测是否需要“窗口滚动”，从参数服务器获取以下参数
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  private_nh.param("rolling_window", rolling_window, false);
  private_nh.param("track_unknown_space", track_unknown_space, false);
  private_nh.param("always_send_full_costmap", always_send_full_costmap, false);
  //4 接下来创建一个LayeredCostmap类实例layered_costmap_，作为规划器使用的主地图，并通过它管理各层地图。
  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);
   //如果没有这一项参数，重新设置旧参数
  if (!private_nh.hasParam("plugins"))
  {
    loadOldParameters(private_nh);
  } else {
    warnForOldParameters(private_nh);
  }
  // 5） 加载plugins 读取plugins参数中指定的costmap plugin， 加载并初始化
  /**
   * 在参数服务器上获取“plugins”参数，这里得到的插件即为各层子地图。
   * 每层子地图使用Pluginlib（ROS插件机制）来实例化，各个层可以被独立的编译，
   * 且允许使用C++接口对Costmap做出任意的改变。循
   * 环将每个plugin即每层子地图添加进layered_costmap_类的指针组对象中，
   * 并对每层地图调用其initialize类方法，初始化各层地图。这个函数定义在Layer类中，
   * 它向各层地图“通知”主地图的存在，并调用oninitialize类方法（Layer类中的虚函数，
   * 被定义在各层地图中）。
   */
  //加入各个层次的地图
  if (private_nh.hasParam("plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      ////从my_list里获取地图名称和种类
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      ROS_INFO("%s: Using plugin \"%s\"", name_.c_str(), pname.c_str());

      copyParentParameters(pname, type, private_nh);
      /**
       * initialize()会调用相应layer的onInitialize()函数
       * 每一个layer对象也有一个LayeredCostmap* layered_costmap_指针。会把此指针赋值为
      * 第4)步Costmap2DROS中的LayeredCostmap* layered_costmap_对象
      * StaticLayer和VoxelLayer有自己的costmap。 InflationLayer没有自己的costmap，
      * 它直接访问LayeredCostmap中的costmap
      * Costmap2DROS调用父类Layer的initialize()进行初始化。
      * Initialize()最终调用每一个子类的onInitialzie() 函数完成每一个layer自己的初始化。
       */
      //这行会创建一个以type为类类型的实例变量，然后让plugin这个指针指向这个实例
      boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
      layered_costmap_->addPlugin(plugin);
      //实际执行的是plugin调用的父类Layer的方法initialize
      plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  //6)  订阅和发布topic /move_base/global_costmap/footprint
  std::string topic_param, topic;
  if (!private_nh.searchParam("footprint_topic", topic_param))
  {
    topic_param = "footprint_topic";
  }
  // 7设置robot的footprint 从配置参数footprint或者robot_radius中读取
  /**
   * 订阅footprint话题，回调函数为setUnpaddedRobotFootprintPolygon。
   * 当话题上收到footprint时，回调函数会将接收到的footprint
   * 根据参数footprint_padding_的值进行“膨胀”，
   * 得到“膨胀”后的padded_footprint_，传递给各级地图。
   */
  private_nh.param(topic_param, topic, std::string("footprint"));
  footprint_sub_ = private_nh.subscribe(topic, 1, &Costmap2DROS::setUnpaddedRobotFootprintPolygon, this);

  if (!private_nh.searchParam("published_footprint_topic", topic_param))
  {
    topic_param = "published_footprint";
  }
 //该话题上将发布的内容是根据机器人当前位置计算出来的实时footprint的位置。
  private_nh.param(topic_param, topic, std::string("footprint"));  // TODO: revert to oriented_footprint in N-turtle
  footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>(topic, 1);

  setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh));
  // 8） 创建Costmap2DPublisher
  publisher_ = new Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap",
                                      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving
  // 9）创建定时器，每隔0.1秒判断robot是否处于运动中
  //创建地图更新线程的控制量
  robot_stopped_ = false;
  //回调函数movementCB通过比较前后两个pose的差来判断机器人是否在移动
  timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);
  // 10）创建mapUpdateLoop线程  在Costmap2DROS::reconfigureCB（）会创建线程。
  dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
  //开启动态参数配置，它的回调函数Costmap2DROS::reconfigureCB会在节点启动时运行一次，加载.cfg文件的配置参数。
  //这个函数给对应参数赋值，并创建了一个Costmap2DROS::mapUpdateLoop函数的线程，即地图更新线程。
  dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1,
                                                                              _2);
  dsrv_->setCallback(cb);
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  timer_.stop();

  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  delete layered_costmap_;
  delete dsrv_;
}

void Costmap2DROS::loadOldParameters(ros::NodeHandle& nh)
{
  ROS_WARN("%s: Parameter \"plugins\" not provided, loading pre-Hydro parameters", name_.c_str());
  bool flag;
  std::string s;
  std::vector < XmlRpc::XmlRpcValue > plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  SuperValue super_array;
  //静态层
  if (nh.getParam("static_map", flag) && flag)
  {
    map["name"] = XmlRpc::XmlRpcValue("static_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::StaticLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    ros::NodeHandle map_layer(nh, "static_layer");
    move_parameter(nh, map_layer, "map_topic");
    move_parameter(nh, map_layer, "unknown_cost_value");
    move_parameter(nh, map_layer, "lethal_cost_threshold");
    move_parameter(nh, map_layer, "track_unknown_space", false);
  }
  //障碍物层，体素层
  ros::NodeHandle obstacles(nh, "obstacle_layer");
  if (nh.getParam("map_type", s) && s == "voxel")
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::VoxelLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    move_parameter(nh, obstacles, "origin_z");
    move_parameter(nh, obstacles, "z_resolution");
    move_parameter(nh, obstacles, "z_voxels");
    move_parameter(nh, obstacles, "mark_threshold");
    move_parameter(nh, obstacles, "unknown_threshold");
    move_parameter(nh, obstacles, "publish_voxel_map");
  }
  else
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  move_parameter(nh, obstacles, "track_unknown_space", true);
  nh.param("observation_sources", s, std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source)
  {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");
  //膨胀层
  ros::NodeHandle inflation(nh, "inflation_layer");
  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  nh.setParam("plugins", super_array);
}

void Costmap2DROS::copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh)
{
  ros::NodeHandle target_layer(nh, plugin_name);

  if(plugin_type == "costmap_2d::StaticLayer")
  {
    move_parameter(nh, target_layer, "map_topic", false);
    move_parameter(nh, target_layer, "unknown_cost_value", false);
    move_parameter(nh, target_layer, "lethal_cost_threshold", false);
    move_parameter(nh, target_layer, "track_unknown_space", false);
  }
  else if(plugin_type == "costmap_2d::VoxelLayer")
  {
    move_parameter(nh, target_layer, "origin_z", false);
    move_parameter(nh, target_layer, "z_resolution", false);
    move_parameter(nh, target_layer, "z_voxels", false);
    move_parameter(nh, target_layer, "mark_threshold", false);
    move_parameter(nh, target_layer, "unknown_threshold", false);
    move_parameter(nh, target_layer, "publish_voxel_map", false);
  }
  else if(plugin_type == "costmap_2d::ObstacleLayer")
  {
    move_parameter(nh, target_layer, "max_obstacle_height", false);
    move_parameter(nh, target_layer, "raytrace_range", false);
    move_parameter(nh, target_layer, "obstacle_range", false);
    move_parameter(nh, target_layer, "track_unknown_space", false);
  }
  else if(plugin_type == "costmap_2d::InflationLayer")
  {
    move_parameter(nh, target_layer, "cost_scaling_factor", false);
    move_parameter(nh, target_layer, "inflation_radius", false);
  }
}

void Costmap2DROS::warnForOldParameters(ros::NodeHandle& nh)
{
  checkOldParam(nh, "static_map");
  checkOldParam(nh, "map_type");
}

void Costmap2DROS::checkOldParam(ros::NodeHandle& nh, const std::string &param_name){
  if(nh.hasParam(param_name)){
    ROS_WARN("%s: Pre-Hydro parameter \"%s\" unused since \"plugins\" is provided", name_.c_str(), param_name.c_str());
  }
}
/**
 * 
 */
void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void Costmap2DROS::readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                                           const costmap_2d::Costmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::Point> new_footprint;
    if (makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        ROS_ERROR("Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(makeFootprintFromRadius(new_config.robot_radius));
  }
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::movementCB(const ros::TimerEvent &event)
{
  // don't allow configuration to happen while this check occurs
  // boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  geometry_msgs::PoseStamped new_pose;

  if (!getRobotPose(new_pose))
  {
    ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else
  {
    old_pose_ = new_pose;

    robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
                                   old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
                                       new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
                     (tf2::Quaternion(old_pose_.pose.orientation.x,
                                      old_pose_.pose.orientation.y,
                                      old_pose_.pose.orientation.z,
                                      old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
                                          new_pose.pose.orientation.y,
                                          new_pose.pose.orientation.z,
                                          new_pose.pose.orientation.w)) < 1e-3);
  }
}
// 启动一个线程处理工作流程 Costmap2DROS初始化完成，启动mapUpdateLoop线程更新地图
//这个函数循环调用UpdateMap函数，更新地图。并以publish_cycle为周期，发布更新后的地图。
void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  ros::NodeHandle nh;
  ros::Rate r(frequency);
  ROS_INFO("mapUpdateLoop frequency:%f,map_update_thread_shutdown_:%d",frequency,map_update_thread_shutdown_);
  while (nh.ok() && !map_update_thread_shutdown_)
  {
    #ifdef HAVE_SYS_TIME_H
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    #endif
    // 更新地图
    updateMap();

    #ifdef HAVE_SYS_TIME_H
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    //计算地图更新时间
    t_diff = end_t - start_t;
    ROS_DEBUG("Map update time: %.9f", t_diff);
    #endif
    //  2） publisher_->updateBounds(x0, xn, y0, yn);
    ROS_INFO("publish_cycle.toSec():%d,layered_costmap_->isInitialized():%d",publish_cycle.toSec(),layered_costmap_->isInitialized());
    if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      // //更新地图边界及发布
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      ROS_INFO("getBounds:%d,%d,%d,%d",x0,xn,y0,yn);
      publisher_->updateBounds(x0, xn, y0, yn);
      ROS_INFO("updateBounds:%d,%d,%d,%d",x0,xn,y0,yn);

      ros::Time now = ros::Time::now();
      //   3） 发布costmap到/move_base/global_costmap/costmap
      //   和/move_base/global_costmap/costmap_updates
      if (last_publish_ + publish_cycle < now)
      {
        //发布
        ROS_INFO("comeinto publisher_->publishCostmap();");
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}
//更新地图
/**
 *首先调用类内getRobotPose函数，通过tf转换，将机器人底盘系的坐标转换到global系，
 得到机器人的位姿。然后通过layered_costmap_调用LayeredCostmap类的updateMap函数，更新地图。 
 */
void Costmap2DROS::updateMap()
{
  ROS_INFO("-----1----start updateMap. stop_updates_:%d",stop_updates_);
  if (!stop_updates_)
  {
    // get global pose
    //>首先获取机器人的当前全局位姿(x, y, yaw)
    geometry_msgs::PoseStamped pose;
    ROS_INFO("updateMap pose:x:%f,y:f,yaw:%f",pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation));
    if (getRobotPose (pose))
    {
      ROS_INFO("comeinto getRobotPose,x:%f,y:%f.yaw:%f",pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation));
      double x = pose.pose.position.x,
             y = pose.pose.position.y,
             yaw = tf2::getYaw(pose.pose.orientation);
      //调用layered_costmap_->updateMap(x, y, yaw);
      ////调用layered_costmap_的updateMap函数，参数是机器人位姿
      ROS_INFO("comeinto layered_costmap_ updateMap!");
      layered_costmap_->updateMap(x, y, yaw);
      ROS_INFO("comeinto layered_costmap_ updateMap finish!");
      //更新机器人的实时足迹，通过footprint_pub_发布
      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      // > 发布机器人的footprint
      footprint_pub_.publish(footprint);

      initialized_ = true;
    }
    else
    {
       ROS_INFO("getRobotPose failed!");
    }
  }
}
//start函数在Movebase中被调用，这个函数对各层地图插件调用activate函数，激活各层地图。
void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  ////检查地图是否暂停或者停止
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    //如果停止，需要重新订阅话题
    //“Layer”是一个类，active是其中一个类方法
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  //在costmap被重新初始化前阻塞..meaning one update cycle has run
  ros::Rate r(100.0);
  while (ros::ok() && !initialized_)
    r.sleep();
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::PoseStamped& global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    // use current time if possible (makes sure it's not in the future)
    if (tf_.canTransform(global_frame_, robot_base_frame_, current_time))
    {
      geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, robot_base_frame_, current_time);
      tf2::doTransform(robot_pose, global_pose, transform);
    }
    // use the latest otherwise
    else
    {
      tf_.transform(robot_pose, global_pose, global_frame_);
    }
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
                     padded_footprint_, oriented_footprint);
}

}  // namespace costmap_2d
