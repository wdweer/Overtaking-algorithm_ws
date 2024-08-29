#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hmcl_msgs/LaneArray.h>
#include <hmcl_msgs/Lane.h>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
class ModelPredictive {
public:
    ModelPredictive();
    void detected_object_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);
    void globalTrajCallback(const hmcl_msgs::LaneArray::ConstPtr& msg);
    void globalTrajCallback1(const hmcl_msgs::Lane::ConstPtr& msg);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void run();
    void detected_object_kinematic(int obj);
    void Intuitive_Artificial_potential_field_2();
    std::pair<double, double> cartesian_to_frenet(const std::vector<std::vector<double>>& centerline, const std::vector<double>& point);
    std::pair<double, double> frenet_to_cartesian(const std::vector<std::vector<double>>& centerline, double s, double l);
    void OG_PUB();
    visualization_msgs::MarkerArray visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints);
    void Bezier_Curve(int obj);
    std::vector<std::pair<double, double>> calc_curve(int num_points);
    void Trajectory_Generation();
private:
    std::vector<int> obj; 
    autoware_msgs::DetectedObjectArray objects_data;
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber global_traj_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber global_traj1_sub_;
    ros::Publisher overtaking_traj_pub_;
    ros::Publisher marker_pub_;

    int model_predicted_num;
    double dt;
    std::vector<double> control;
    std::vector<double> state;
    double qx, qy, qz, qw;  
    double x, y;
    std::map<int, std::vector<double>> target_vel_x, target_vel_y; // Assuming use of id as key
    std::map<int, double> target_x, target_y, target_velocity_x, target_velocity_y, target_orientation_x, target_orientation_y, target_orientation_z, target_orientation_w, target_yaw_veh, target_angular_z_veh;
    std::vector<double> cx, cy, cqx, cqy, cqz, cqw;
    std::map<int, std::vector<double>> target_veh_dic_x, target_veh_dic_y;
    std::map<int, std::vector<double>> range_point_x, range_point_y, repulsed_potential_field_point_x, repulsed_potential_field_point_y;

    std::vector<double> first_point, second_point, third_point, fourth_point;
    std::vector<std::pair<double, double>> B;

    double target_velocity;
    double target_angular_z;
    double gain;
    double sigma;
    double radius;
    double repulsed_s, repulsed_d;
    double repulsed_x, repulsed_y;
      
};

ModelPredictive::ModelPredictive() : model_predicted_num(5), dt(0.1) {
    target_sub_ = nh_.subscribe("/tracking_side/objects", 1, &ModelPredictive::detected_object_callback, this);
    global_traj_sub_ = nh_.subscribe("/optimal_traj", 1, &ModelPredictive::globalTrajCallback, this);
    global_traj1_sub_ = nh_.subscribe("/local_traj", 1, &ModelPredictive::globalTrajCallback1, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &ModelPredictive::currentPoseCallback, this);
    overtaking_traj_pub_ = nh_.advertise<hmcl_msgs::Lane>("/local_traj1", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/over_traj_viz", 1);
}

void ModelPredictive::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;
    x = msg->pose.position.x;
    y = msg->pose.position.y;
}

void ModelPredictive::detected_object_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {
    std::cout << "Object_detected" << std::endl;
    target_veh_dic_x.clear();
    target_veh_dic_y.clear();
    obj.clear();
    objects_data = *msg;  
    for (const auto& obj : msg->objects) {
        int obj_key = obj.id; // Using ID as key

        target_velocity_x[obj_key] = obj.velocity.linear.x;
        target_velocity_y[obj_key] = obj.velocity.linear.y;
        target_x[obj_key] = obj.pose.position.x;
        target_y[obj_key] = obj.pose.position.y;
        target_orientation_x[obj_key] = obj.pose.orientation.x;
        target_orientation_y[obj_key] = obj.pose.orientation.y;
        target_orientation_z[obj_key] = obj.pose.orientation.z;
        target_orientation_w[obj_key] = obj.pose.orientation.w;
        target_yaw_veh[obj_key] = obj.velocity.angular.z;

        detected_object_kinematic(obj_key); // Custom function to handle object kinematics based on label or other identifiers
        
        // Additional logic for velocity history and LSTM model might be included here.
    }

    Intuitive_Artificial_potential_field_2(); // Assuming another method handles potential field calculation
}

void ModelPredictive::detected_object_kinematic(int obj) {
    double model_prediction_x = target_x[obj];
    double target_vel_x = target_velocity_x[obj];
    double target_yaw = target_yaw_veh[obj];

    double model_prediction_y = target_y[obj];
    double target_vel_y = target_velocity_y[obj];
    double target_angular_z = target_angular_z_veh[obj];

    std::vector<double> predicted_x, predicted_y;

    for (int i = 0; i < model_predicted_num; ++i) {
        predicted_x.push_back(model_prediction_x);
        model_prediction_x += target_vel_x;
        target_yaw += target_angular_z;
        target_vel_x = pow((pow(target_vel_x,2)+pow(target_vel_y,2)),1/2) * std::cos(target_yaw);
    }
    target_veh_dic_x[obj] = predicted_x;

    for (int j = 0; j < model_predicted_num; ++j) {
        predicted_y.push_back(model_prediction_y);
        model_prediction_y += target_vel_y;
        target_yaw += target_angular_z;  // Note: This recalculates target_yaw; ensure this is the desired behavior
        target_vel_y = pow((pow(target_vel_x,2)+pow(target_vel_y,2)),1/2) * std::sin(target_yaw);
    }
    target_veh_dic_y[obj] = predicted_y;
}
void ModelPredictive::globalTrajCallback1(const hmcl_msgs::Lane::ConstPtr& data) {
    // Clear the vectors before populating them
    std::cout << "local_traj" << std::endl;
    cx.clear();
    cy.clear();
    cqx.clear();
    cqy.clear();
    cqz.clear();
    cqw.clear();

    // Iterate through the waypoints and populate the vectors
    for (const auto& waypoint : data->waypoints) {
        cx.push_back(waypoint.pose.pose.position.x);
        cy.push_back(waypoint.pose.pose.position.y);
        cqx.push_back(waypoint.pose.pose.orientation.x);
        cqy.push_back(waypoint.pose.pose.orientation.y);
        cqz.push_back(waypoint.pose.pose.orientation.z);
        cqw.push_back(waypoint.pose.pose.orientation.w);
    }
}
void ModelPredictive::globalTrajCallback(const hmcl_msgs::LaneArray::ConstPtr& msg) {
    // Clear previous data
    std::cout << "global_traj" << std::endl;
    cx.clear();
    cy.clear();
    cqx.clear();
    cqy.clear();
    cqz.clear();
    cqw.clear();

    // Iterate through all lanes and waypoints in the received message
    for (const auto& lane : msg->lanes) {
        for (const auto& waypoint : lane.waypoints) {
            cx.push_back(waypoint.pose.pose.position.x);
            cy.push_back(waypoint.pose.pose.position.y);
            cqx.push_back(waypoint.pose.pose.orientation.x);
            cqy.push_back(waypoint.pose.pose.orientation.y);
            cqz.push_back(waypoint.pose.pose.orientation.z);
            cqw.push_back(waypoint.pose.pose.orientation.w);
        }
    }
}
std::pair<double, double> ModelPredictive::cartesian_to_frenet(const std::vector<std::vector<double>>& centerline, const std::vector<double>& point) {
    Eigen::MatrixXd centerline_mat(centerline.size(), 2);
    for (size_t i = 0; i < centerline.size(); ++i) {
        centerline_mat(i, 0) = centerline[i][0];
        centerline_mat(i, 1) = centerline[i][1];
    }

    Eigen::MatrixXd diffs = centerline_mat.bottomRows(centerline.size() - 1) - centerline_mat.topRows(centerline.size() - 1);
    Eigen::VectorXd dists = diffs.rowwise().norm();
    Eigen::VectorXd arclength(dists.size() + 1);
    arclength(0) = 0.0;
    for (int i = 0; i < dists.size(); ++i) {
        arclength(i + 1) = arclength(i) + dists(i);
    }

    Eigen::Vector2d point_vec(point[0], point[1]);
    double min_dist = std::numeric_limits<double>::infinity();
    double s = 0, l = 0;

    for (int i = 0; i < diffs.rows(); ++i) {
        Eigen::Vector2d p1 = centerline_mat.row(i);
        Eigen::Vector2d p2 = centerline_mat.row(i + 1);

        Eigen::Vector2d line_vec = p2 - p1;
        Eigen::Vector2d point_to_p1 = point_vec - p1;
        double line_len = line_vec.norm();
        double proj_length = point_to_p1.dot(line_vec) / line_len;
        Eigen::Vector2d proj_point = p1 + (proj_length / line_len) * line_vec;

        double dist = (point_vec - proj_point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            s = arclength(i) + proj_length;
            l = dist;
        }
    }

    return std::make_pair(s, l);
}

std::pair<double, double> ModelPredictive::frenet_to_cartesian(const std::vector<std::vector<double>>& centerline, double s, double l) {
    Eigen::MatrixXd centerline_mat(centerline.size(), 2);
    for (size_t i = 0; i < centerline.size(); ++i) {
        centerline_mat(i, 0) = centerline[i][0];
        centerline_mat(i, 1) = centerline[i][1];
    }

    Eigen::MatrixXd diffs = centerline_mat.bottomRows(centerline.size() - 1) - centerline_mat.topRows(centerline.size() - 1);
    Eigen::VectorXd dists = diffs.rowwise().norm();
    Eigen::VectorXd arclength(dists.size() + 1);
    arclength(0) = 0.0;
    for (int i = 0; i < dists.size(); ++i) {
        arclength(i + 1) = arclength(i) + dists(i);
    }

    int segment_index = std::lower_bound(arclength.data(), arclength.data() + arclength.size(), s) - arclength.data() - 1;
    if (segment_index < 0) {
        segment_index = 0;
    } else if (segment_index >= centerline.size() - 1) {
        segment_index = centerline.size() - 2;
    }

    Eigen::Vector2d p1 = centerline_mat.row(segment_index);
    Eigen::Vector2d p2 = centerline_mat.row(segment_index + 1);

    Eigen::Vector2d segment_vector = p2 - p1;
    double segment_length = dists(segment_index);
    Eigen::Vector2d segment_unit_vector = segment_vector / segment_length;

    Eigen::Vector2d base_point = p1 + segment_unit_vector * (s - arclength(segment_index));

    Eigen::Vector2d normal_vector(-segment_unit_vector(1), segment_unit_vector(0));

    Eigen::Vector2d cartesian_point = base_point + normal_vector * l;

    return std::make_pair(cartesian_point(0), cartesian_point(1));
}

void ModelPredictive::OG_PUB() {
    hmcl_msgs::Lane trajectory;
    int i = 0;

    while (i < cx.size()) {
        hmcl_msgs::Waypoint waypoint;
        waypoint.pose.pose.position.x = cx[i];
        waypoint.pose.pose.position.y = cy[i];
        waypoint.pose.pose.orientation.x = cqx[i];
        waypoint.pose.pose.orientation.y = cqy[i];
        waypoint.pose.pose.orientation.z = cqz[i];
        waypoint.pose.pose.orientation.w = cqw[i];
        trajectory.waypoints.push_back(waypoint);
        i++;
    }
    overtaking_traj_pub_.publish(trajectory);

    // Visualize the local path using markers
    visualization_msgs::MarkerArray marker_array = visualize_local_path(trajectory.waypoints);
    marker_pub_.publish(marker_array);
    std::cout << "work2222" << std::endl;
}

visualization_msgs::MarkerArray ModelPredictive::visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "local_path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = waypoints[i].pose.pose.position;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

void ModelPredictive::Bezier_Curve(int obj) {
    std::cout << "work!" << std::endl;

    std::vector<double> repulsed_x = repulsed_potential_field_point_x[obj];
    std::vector<double> repulsed_y = repulsed_potential_field_point_y[obj];

    int num = repulsed_x.size();

    std::vector<int> indices = {(num - 1) / 3, (num - 1) * 2 / 3};
    first_point = {repulsed_x[0], repulsed_y[0]};
    second_point = {repulsed_x[indices[0]], repulsed_y[indices[0]]};
    third_point = {repulsed_x[indices[1]], repulsed_y[indices[1]]};
    fourth_point = {repulsed_x[num - 1], repulsed_y[num - 1]};

    B = calc_curve(100);

    Trajectory_Generation();
}

std::vector<std::pair<double, double>> ModelPredictive::calc_curve(int num_points) {
    std::vector<std::pair<double, double>> curve_points;
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = std::pow(1 - t, 3) * first_point[0] + 3 * std::pow(1 - t, 2) * t * second_point[0] +
                   3 * (1 - t) * std::pow(t, 2) * third_point[0] + std::pow(t, 3) * fourth_point[0];
        double y = std::pow(1 - t, 3) * first_point[1] + 3 * std::pow(1 - t, 2) * t * second_point[1] +
                   3 * (1 - t) * std::pow(t, 2) * third_point[1] + std::pow(t, 3) * fourth_point[1];
        curve_points.emplace_back(x, y);
    }
    return curve_points;
}

void ModelPredictive::Trajectory_Generation() {
    hmcl_msgs::Lane trajectory_msg;  // Trajectory message to be published

    // Ensure B has at least two points to calculate orientation
    if (B.size() < 2) {
        ROS_WARN("Not enough points in B to generate a trajectory.");
        return;
    }

    for (size_t i = 0; i < B.size(); ++i) {
        hmcl_msgs::Waypoint wp;
        wp.pose.pose.position.x = B[i].first;  // Assuming B is a vector of pairs or a similar structure
        wp.pose.pose.position.y = B[i].second;
        wp.pose.pose.position.z = 0.0;  // Assuming z is constant

        // Calculate orientation based on the next waypoint
        if (i < B.size() - 1) {
            double delta_x = B[i + 1].first - B[i].first;
            double delta_y = B[i + 1].second - B[i].second;
            double yaw = std::atan2(delta_y, delta_x);

            // Convert yaw to quaternion
            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw);  // Roll and Pitch are 0, Yaw is calculated

            wp.pose.pose.orientation = tf2::toMsg(quat);
        } else {
            // For the last waypoint, you might need to set a default orientation
            wp.pose.pose.orientation.w = 1.0;  // Default to no rotation (identity quaternion)
        }

        trajectory_msg.waypoints.push_back(wp);
    }

    // Publish the trajectory
    overtaking_traj_pub_.publish(trajectory_msg);

    // Visualize the local path using markers
    visualization_msgs::MarkerArray marker_array = visualize_local_path(trajectory_msg.waypoints);
    marker_pub_.publish(marker_array);

    ROS_INFO("Trajectory generation and publishing completed.");
}
void ModelPredictive::Intuitive_Artificial_potential_field_2() {
    auto start_time = std::clock();
    double center_s, center_d, s1, d1, s, d;
    gain = 10;
    target_velocity = 300;
    sigma = 1 / target_velocity;
    radius = round(std::sqrt(1 / sigma) + 1) * 10000000;

    std::map<int, std::vector<std::vector<double>>> potential_field_point;

    for (const auto& obj : objects_data.objects) {
        int obj_key =obj.id;

        potential_field_point[obj_key] = {};
        range_point_x[obj_key] = {};
        range_point_y[obj_key] = {};
        repulsed_potential_field_point_x[obj_key] = {};
        repulsed_potential_field_point_y[obj_key] = {};

        for (size_t i = 0; i < cx.size(); ++i) {
            double distance = std::sqrt(std::pow(target_veh_dic_x[obj_key][2] - cx[i], 2) + 
                                        std::pow(target_veh_dic_y[obj_key][2] - cy[i], 2));
            if (distance < radius) {
                potential_field_point[obj_key].push_back({cx[i], cy[i]});
            }
        }

        if (!potential_field_point[obj_key].empty()) {
            std::vector<double> point = potential_field_point[obj_key][potential_field_point[obj_key].size() / 2];
            std::tie(center_s, center_d) = cartesian_to_frenet(potential_field_point[obj_key], point);
            std::tie(s1, d1) = cartesian_to_frenet(potential_field_point[obj_key], 
                                                {target_veh_dic_x[obj_key][2], target_veh_dic_y[obj_key][2]});

            for (const auto& j : potential_field_point[obj_key]) {
                std::tie(s, d) = cartesian_to_frenet(potential_field_point[obj_key], j);
                for (int i = 0; i < model_predicted_num; ++i) {
                    if (d1 >= 0) {
                        d -= gain * std::sqrt(std::max(0.0, 1 - sigma * std::pow(s - s1, 2)));
                    } else {
                        d += gain * std::sqrt(std::max(0.0, 1 - sigma * std::pow(s - s1, 2)));
                    }
                }

                repulsed_s = s;
                repulsed_d = d;

                std::tie(repulsed_x, repulsed_y) = frenet_to_cartesian(potential_field_point[obj_key], repulsed_s, repulsed_d);
                repulsed_potential_field_point_x[obj_key].push_back(repulsed_x);
                repulsed_potential_field_point_y[obj_key].push_back(repulsed_y);
            }
            Bezier_Curve(obj_key);
        }
    }

}
void ModelPredictive::run() {
    ros::Rate loop_rate(10);  // Adjust the rate as necessary
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ModelPredictive");
    ModelPredictive model_predictive;
    model_predictive.run();
    return 0;
}
