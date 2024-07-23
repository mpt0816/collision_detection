#include "collision_detect.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

void convertPath(std::string str_namespace, const Path& path, size_t index, visualization_msgs::Marker& outputs);
void convertOgm(std::string str_namespace, const Ogm& ogm, visualization_msgs::Marker& outputs);
void convertObstacle(std::string str_namespace, const Obstacles& obstacles, visualization_msgs::MarkerArray& outputs);
void convertRectangle(std::string str_namespace, CollisionDetect& collision_detect,
        visualization_msgs::MarkerArray& outputs);
void convertCircle(std::string str_namespace, CollisionDetect& collision_detect,
             visualization_msgs::MarkerArray& outputs);


int main(int argc, char * argv[]) {
  std::cout << "<<--------- collision_detect --------->" << std::endl;

  ros::init(argc, argv, "collision_detect");
  ros::NodeHandle nh;

  CollisionDetect collision_detect;
  collision_detect.ComputeCollision();
  collision_detect.printResult();

  ros::Publisher pub_path_maker = nh.advertise<visualization_msgs::Marker>("path_marker", 1, true);
  ros::Publisher pub_ogm_maker  = nh.advertise<visualization_msgs::Marker>("ogm_marker", 1, true);
  ros::Publisher pub_obstacle_maker = nh.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 1, true);
  ros::Publisher pub_veh_rectangle = nh.advertise<visualization_msgs::MarkerArray>("veh_rectangle", 1, true);
  ros::Publisher pub_veh_circle = nh.advertise<visualization_msgs::MarkerArray>("veh_circle", 1, true);

  ros::Rate rate(10);
  while (ros::ok()) {
      visualization_msgs::Marker path_marker;
      visualization_msgs::Marker ogm_marker;
      visualization_msgs::MarkerArray obstacles_marker;
      visualization_msgs::MarkerArray veh_rectangle;
      visualization_msgs::MarkerArray veh_circle;

      convertPath("path", collision_detect.getPath(), collision_detect.getCollionIndex(), path_marker);
      convertOgm("ogm", collision_detect.getOgm(), ogm_marker);
      convertObstacle("obstacles", collision_detect.getObstacles(), obstacles_marker);
      convertRectangle("veh_rectangle", collision_detect, veh_rectangle);
      convertCircle("veh_circle", collision_detect, veh_circle);


      pub_path_maker.publish(path_marker);
      pub_ogm_maker.publish(ogm_marker);
      pub_obstacle_maker.publish(obstacles_marker);
      pub_veh_rectangle.publish(veh_rectangle);
      pub_veh_circle.publish(veh_circle);
      rate.sleep();
  }

  return 0;
}

void convertPath(std::string str_namespace, const Path& path, size_t index, visualization_msgs::Marker& outputs) {
    outputs.header.frame_id = "/map";
    outputs.header.stamp = ros::Time::now();
    outputs.lifetime = ros::Duration();
    outputs.ns = str_namespace;
    outputs.id = 0;
    outputs.type = visualization_msgs::Marker::POINTS;
    outputs.action = visualization_msgs::Marker::ADD;

    outputs.scale.x = 0.2;
    outputs.scale.y = 0.2;
    outputs.scale.z = 0.0;

    for (size_t i =0; i < path.size(); ++i) {
        geometry_msgs::Point pt;
        std_msgs::ColorRGBA color;
        pt.x = path[i].pt.x;
        pt.y = path[i].pt.y;
        pt.z = 0.0;
        outputs.points.push_back(pt);

        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        if (i == index) {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
        }
        outputs.colors.push_back(color);
    }
}

void convertOgm(std::string str_namespace, const Ogm& ogm, visualization_msgs::Marker& outputs) {
    outputs.header.frame_id = "/map";
    outputs.header.stamp = ros::Time::now();
    outputs.lifetime = ros::Duration();
    outputs.ns = str_namespace;
    outputs.id = 0;
    outputs.type = visualization_msgs::Marker::POINTS;
    outputs.action = visualization_msgs::Marker::ADD;
    outputs.scale.x = 0.25;
    outputs.scale.y = 0.25;
    outputs.scale.z = 0.0;

    for (auto& ogm_pt : ogm) {
        geometry_msgs::Point pt;
        std_msgs::ColorRGBA color;
        pt.x = ogm_pt.pt.x;
        pt.y = ogm_pt.pt.y;
        pt.z = 0.0;
        outputs.points.push_back(pt);

        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        outputs.colors.push_back(color);
    }

}

void convertObstacle(std::string str_namespace, const Obstacles& obstacles, visualization_msgs::MarkerArray& outputs) {

    int i = 0;
    for (auto& obstacle : obstacles) {
        visualization_msgs::Marker output;
        output.header.frame_id = "/map";
        output.header.stamp = ros::Time::now();
        output.lifetime = ros::Duration();
        output.ns = str_namespace;
        output.id = i;
        output.type = visualization_msgs::Marker::LINE_STRIP;
        output.action = visualization_msgs::Marker::ADD;
        output.color.r = 1.0;
        output.color.g = 1.0;
        output.color.b = 0.0;
        output.color.a = 1.0;

        output.scale.x = 0.1;
        output.scale.y = 0.1;
        output.scale.z = 0.0;
        for (auto& pt : obstacle.contour) {
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.0;
            output.points.push_back(p);
        }
        geometry_msgs::Point p;
        p.x = obstacle.contour[0].x;
        p.y = obstacle.contour[0].y;
        p.z = 0.0;
        output.points.push_back(p);
        outputs.markers.push_back(output);
        i = i + 1;
    }
}

void convertRectangle(std::string str_namespace, CollisionDetect& collision_detect,
             visualization_msgs::MarkerArray& outputs) {
    Path path = collision_detect.getPath();
    if (path.empty()) {
        return;
    }

    size_t collision_index = collision_detect.getCollionIndex();
    size_t id = 0;
    // get vehicle box
    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::Marker output;
        PathPoint pose;
        VehicleModel veh_model;
        pose = path.at(i);
        veh_model = collision_detect.getVehModel();
        output.header.frame_id = "/map";
        output.header.stamp = ros::Time::now();
        output.lifetime = ros::Duration();
        output.ns = str_namespace;
        output.id = id;
        output.type = visualization_msgs::Marker::LINE_STRIP;
        output.action = visualization_msgs::Marker::ADD;
        if (i == collision_index) {
            output.color.r = 1.0;
            output.color.g = 0.0;
            output.color.b = 0.0;
            output.color.a = 1.0;

            output.scale.x = 0.25;
            output.scale.y = 0.25;
            output.scale.z = 0.0;
        } else {
            output.color.r = 1.0;
            output.color.g = 1.0;
            output.color.b = 1.0;
            output.color.a = 0.05;

            output.scale.x = 0.05;
            output.scale.y = 0.05;
            output.scale.z = 0.0;
        }


        veh_model.rectangleRep(pose);
        for (auto& pt : veh_model.rect_contour) {
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.0;
            output.points.push_back(p);
        }
        geometry_msgs::Point p;
        p.x = veh_model.rect_contour.at(0).x;
        p.y = veh_model.rect_contour.at(0).y;
        p.z = 0.0;
        output.points.push_back(p);
        outputs.markers.push_back(output);
        id += 1;
    }

}

void convertCircle(std::string str_namespace, CollisionDetect& collision_detect,
                   visualization_msgs::MarkerArray& outputs) {
    Path path = collision_detect.getPath();
    if (path.empty()) {
        return;
    }

    size_t collision_index = collision_detect.getCollionIndex();
    size_t id = 0;
    // get vehicle bounding circle
    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::Marker output;
        PathPoint pose;
        VehicleModel veh_model;
        pose = path.at(i);
        veh_model = collision_detect.getVehModel();
        output.header.frame_id = "/map";
        output.header.stamp = ros::Time::now();
        output.lifetime = ros::Duration();
        output.ns = str_namespace;
        output.id = id;
        output.type = visualization_msgs::Marker::LINE_STRIP;
        output.action = visualization_msgs::Marker::ADD;
        if (i == collision_index) {
            output.color.r = 1.0;
            output.color.g = 0.0;
            output.color.b = 0.0;
            output.color.a = 1.0;

            output.scale.x = 0.2;
            output.scale.y = 0.2;
            output.scale.z = 0.0;
        } else {
            output.color.r = 1.0;
            output.color.g = 1.0;
            output.color.b = 1.0;
            output.color.a = 0.05;

            output.scale.x = 0.05;
            output.scale.y = 0.05;
            output.scale.z = 0.0;
        }


        veh_model.circleRep(pose);
        double radius = veh_model.circle_bounding.radius;
        for (int j = 0; j <= 360; ++j) {
            geometry_msgs::Point pt;
            pt.x = veh_model.circle_bounding.center.x + radius * cos(M_PI * 2 * j / 360.0);
            pt.y = veh_model.circle_bounding.center.y + radius * sin(M_PI * 2 * j / 360.0);
            output.points.push_back(pt);
        }
        outputs.markers.push_back(output);
        id += 1;
    }

    // get vehicle square circle
    for (size_t i = 0; i < path.size(); ++i) {
        PathPoint pose;
        VehicleModel veh_model;
        pose = path.at(i);
        veh_model = collision_detect.getVehModel();
        veh_model.circleRep(pose);

        for (auto& circle : veh_model.circle_square) {
            visualization_msgs::Marker output;
            output.header.frame_id = "/map";
            output.header.stamp = ros::Time::now();
            output.lifetime = ros::Duration();
            output.ns = str_namespace;
            output.id = id;
            output.type = visualization_msgs::Marker::LINE_STRIP;
            output.action = visualization_msgs::Marker::ADD;
            if (i == collision_index) {
                output.color.r = 1.0;
                output.color.g = 0.0;
                output.color.b = 0.0;
                output.color.a = 1.0;

                output.scale.x = 0.2;
                output.scale.y = 0.2;
                output.scale.z = 0.0;
            } else {
                output.color.r = 1.0;
                output.color.g = 1.0;
                output.color.b = 1.0;
                output.color.a = 0.05;

                output.scale.x = 0.05;
                output.scale.y = 0.05;
                output.scale.z = 0.0;
            }
            for (int j = 0; j <= 360; ++j) {
                geometry_msgs::Point pt;
                pt.x = circle.center.x + circle.radius * cos(M_PI * 2 * j / 360.0);
                pt.y = circle.center.y + circle.radius * sin(M_PI * 2 * j / 360.0);
                output.points.push_back(pt);
            }
            outputs.markers.push_back(output);
            id += 1;
        }
    }

    // get vehicle rectangle circle
    for (size_t i = 0; i < path.size(); ++i) {
        PathPoint pose;
        VehicleModel veh_model;
        pose = path.at(i);
        veh_model = collision_detect.getVehModel();
        veh_model.circleRep(pose);

        for (auto& circle : veh_model.circle_rectangle) {
            visualization_msgs::Marker output;
            output.header.frame_id = "/map";
            output.header.stamp = ros::Time::now();
            output.lifetime = ros::Duration();
            output.ns = str_namespace;
            output.id = id;
            output.type = visualization_msgs::Marker::LINE_STRIP;
            output.action = visualization_msgs::Marker::ADD;
            if (i == collision_index) {
                output.color.r = 1.0;
                output.color.g = 0.0;
                output.color.b = 0.0;
                output.color.a = 1.0;

                output.scale.x = 0.2;
                output.scale.y = 0.2;
                output.scale.z = 0.0;
            } else {
                output.color.r = 1.0;
                output.color.g = 1.0;
                output.color.b = 1.0;
                output.color.a = 0.05;

                output.scale.x = 0.05;
                output.scale.y = 0.05;
                output.scale.z = 0.0;
            }
            for (int j = 0; j <= 360; ++j) {
                geometry_msgs::Point pt;
                pt.x = circle.center.x + circle.radius * cos(M_PI * 2 * j / 360.0);
                pt.y = circle.center.y + circle.radius * sin(M_PI * 2 * j / 360.0);
                output.points.push_back(pt);
            }
            outputs.markers.push_back(output);
            id += 1;
        }
    }
}

