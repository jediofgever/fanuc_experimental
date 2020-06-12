/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-27 10:16:01
 * @modify date 2020-02-27 10:16:01
 * @desc [description]
 */
#include "SimulationPickandPlace.hh"

// make use gazebo namespace
using namespace gazebo;

/**
 * @brief Construct a new Simulation Pickand Place:: Simulation Pickand Place object
 *
 */
SimulationPickandPlace::SimulationPickandPlace() {
    // initilize ros node handler, to handle usual ros stuff
    nh_ = new ros::NodeHandle();

    // initiazlize TF transfrom listener
    listener_ = new tf::TransformListener();

    // register publisher for ground truth object boxes, mainly for visualzu=izations in RVIZ
    gtBBX_pub_ = nh_->advertise<jsk_recognition_msgs::BoundingBoxArray>("/gt_labels", 1);

    ros_pub_to_see_if_robot_is_in_scan_pose_pub_ = nh_->advertise<std_msgs::Bool>("is_robot_in_scan_pose", 1);
}

/**
 * @brief Destroy the Simulation Pickand Place:: Simulation Pickand Place object
 *
 */
SimulationPickandPlace::~SimulationPickandPlace() {}

/**
 * @brief init world pointer , bind the callback for executing OnUpdate on each cycle
 *
 * @param _parent
 */
void SimulationPickandPlace::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    this->world = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SimulationPickandPlace::OnUpdate, this));
}

/**
 * @brief return a random double ranging from fMin to fMax
 *
 * @param fMin
 * @param fMax
 * @return double
 */
double SimulationPickandPlace::fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

/**
 * @brief Handles object pose, with coordination of SimulationPickPlace
 *
 */
void SimulationPickandPlace::OnUpdate() {
    // model name of Robot itself
    std::string robot_model_name = "lrmate200id7l";

    // Tool link parent
    std::string gripper_link_name = "link_6";

    // base link of robot
    std::string base_link_name = "base_link";

    // Vector of Model pointers, first we will find the object to be picked by their name and then store them in this
    // vector.
    physics::Model_V objects_tobe_picked;

    // To check/get pose of tool link, and continuously check distance to objects to be picked
    physics::LinkPtr gripper_link;

    // To check base link pose
    physics::LinkPtr base_link;

    // stores postion vector of gripper link which is world frame
    ignition::math::Vector3d gripper_link_position_world_frame;

    // stores postion vector of base link which is world frame
    ignition::math::Vector3d base_link_position_world_frame;

    // First lets get all the models in this world
    physics::Model_V all_models_in_this_world = world->Models();

    // Go through all models
    for (int i = 0; i < all_models_in_this_world.size(); i++) {
        // Get current model name
        std::string current_model_name = all_models_in_this_world[i]->GetName();

        // get current model pointer itself
        physics::ModelPtr current_model = all_models_in_this_world[i];

        // Find Robot model and get Robot's links poses (base link and gripper link)
        if (current_model_name == robot_model_name) {
            gripper_link = current_model->GetLink(gripper_link_name);
            gripper_link_position_world_frame = gripper_link->WorldPose().Pos();
            base_link = current_model->GetLink(base_link_name);
            base_link_position_world_frame = base_link->WorldPose().Pos();
        }

        // Find object to be picked and store them into objects_tobe_picked vector. Since all these objects begins with
        // pulley
        // we check if the string starts with this substring which is "pulley"
        if (current_model_name.substr(0, 6) == "pulley") {
            // store this obejct in objects_tobe_picked
            objects_tobe_picked.push_back(current_model);
        }
    }

    // We will publish JSK boxes in RVIZ for visualization
    jsk_recognition_msgs::BoundingBoxArray gt_box_array;
    // These boxes should be in camera frame
    gt_box_array.header.frame_id = "camera_depth_optical_frame";

    // stamp their time to now
    gt_box_array.header.stamp = ros::Time::now();

    // We have objects to be picked, since we do not have a physical gripper , we will attach the object to the Robot
    // gripper
    // using this loop. If distance of gripper link and  current object to be picked gets under a certain value( 0.06)
    // we will attach that object to gripper, and then the gripper will go to the pose of placing , again when gripper
    // link is achieved to placing pose we will de-attach the object
    for (int o = 0; o < objects_tobe_picked.size(); o++) {
        // Current object to be picked
        physics::ModelPtr current_object_tobe_picked = objects_tobe_picked[o];

        // World postion of current object to be picked
        ignition::math::Vector3d current_object_position_world_frame = current_object_tobe_picked->WorldPose().Pos();

        // get distance of gripper link to current object to be picked
        double distance_to_tool_link_meters =
            getDistanceBetweenPoints(gripper_link_position_world_frame, current_object_position_world_frame);

        // If this distance is lower than 0.06 meters, that means the Robot Has detected this object and is trying to
        // pick the object So we enter a pick-place pipeline to perfrom manipulation on this object
        if (distance_to_tool_link_meters < 0.08) {
            // ROS_INFO("PICKING OBJECT");

            // Attach the object to the gripper, with a little offset in z axis
            ignition::math::Pose3d attach_object_to_gripper_pose(
                gripper_link_position_world_frame[0], gripper_link_position_world_frame[1],
                (gripper_link_position_world_frame[2] - 0.04), 0, -1, 0, 0);
            // Set pose of this object to attach_object_to_gripper_pose, so it will look like the gripper has picked the
            // object
            current_object_tobe_picked->SetWorldPose(attach_object_to_gripper_pose, true, true);

            // Turn off gravity
            current_object_tobe_picked->SetGravityMode(false);

            // Where should this object placed ? , first go to default home_pose
            std::vector<ignition::math::Vector3d> placing_poses_vector;
            ignition::math::Vector3d placing_pose;
            placing_pose[0] = 0.550;
            placing_pose[1] = 0.350;
            placing_pose[2] = 0.2;
            placing_poses_vector.push_back(placing_pose);
            placing_pose[0] -= 0.15;
            placing_poses_vector.push_back(placing_pose);
            placing_pose[0] -= 0.15;
            placing_poses_vector.push_back(placing_pose);
            placing_pose[1] += 0.15;
            placing_poses_vector.push_back(placing_pose);
            placing_pose[0] += 0.15;
            placing_poses_vector.push_back(placing_pose);
            placing_pose[0] += 0.15;
            placing_poses_vector.push_back(placing_pose);

            // get the distance of gripper link and place pose, to see if gripper has arrived to place pose, to place
            // the object
            for (int l = 0; l < placing_poses_vector.size(); l++) {
                double distance_to_home_pose =
                    getDistanceBetweenPoints(gripper_link_position_world_frame, placing_poses_vector[l]);

                // if distance lower than 4 cm , then place the object to place_pose
                if (distance_to_home_pose < 0.04) {
                    // ROS_INFO("PLACING OBJECT");
                    ignition::math::Pose3d place_pose(placing_poses_vector[l][0], placing_poses_vector[l][1] + 0.05,
                                                      (placing_poses_vector[l][2] - 0.04), 0, -1, 0, 0);

                    // place the object
                    current_object_tobe_picked->SetWorldPose(place_pose, true, true);

                    // enable back the gravity for object
                    current_object_tobe_picked->SetGravityMode(true);
                }
            }
        }

        // get Bounding box of this object to be picked
        ignition::math::Box box = current_object_tobe_picked->BoundingBox();

        // get properties of box such as center and roataion
        ignition::math::Quaterniond rot(current_object_tobe_picked->WorldPose().Rot());
        ignition::math::Vector3d center;

        // feed object information to geotmetry msgs
        geometry_msgs::Pose pose_in_world_frame;
        center = current_object_tobe_picked->WorldPose().Pos();
        pose_in_world_frame.position.x = center[0];
        pose_in_world_frame.position.y = center[1];
        pose_in_world_frame.position.z = center[2];
        pose_in_world_frame.orientation.x = rot.X();
        pose_in_world_frame.orientation.y = rot.Y();
        pose_in_world_frame.orientation.z = rot.Z();
        pose_in_world_frame.orientation.w = rot.W();

        // This object pose is in world frame but its better if we represent it in camera frame
        tf::Transform pose_in_world_frame_tf;
        tf::poseMsgToTF(pose_in_world_frame, pose_in_world_frame_tf);

        tf::StampedTransform world_to_depth_camera_link_transform;
        // lookup transform (this should be cached, since it’s probably static)
        try {
            listener_->lookupTransform("camera_depth_optical_frame", "world", ros::Time(0.0f),
                                       world_to_depth_camera_link_transform);
        } catch (tf::TransformException ex) {
            // ROS_ERROR("%s", ex.what());
            return;
        }

        // get pose in camera frame
        tf::Transform pose_in_depth_camera_link_tf;
        pose_in_depth_camera_link_tf = world_to_depth_camera_link_transform * pose_in_world_frame_tf;

        geometry_msgs::Pose pose_in_depth_camera_link;
        tf::poseTFToMsg(pose_in_depth_camera_link_tf, pose_in_depth_camera_link);

        // FILL out JSK bounding box msg infor and publish them for RVIZ
        jsk_recognition_msgs::BoundingBox jsk_box_msg;
        jsk_box_msg.header.frame_id = "camera_depth_optical_frame";
        jsk_box_msg.header.stamp = ros::Time::now();
        jsk_box_msg.label = o;
        jsk_box_msg.pose.position = pose_in_depth_camera_link.position;
        jsk_box_msg.pose.orientation = pose_in_depth_camera_link.orientation;

        jsk_box_msg.dimensions.x = 0.112;  // 0.112
        jsk_box_msg.dimensions.y = 0.112;  // 0.112
        jsk_box_msg.dimensions.z = 0.05;   // 0.05
        gt_box_array.boxes.push_back(jsk_box_msg);
    }
    gtBBX_pub_.publish(gt_box_array);
}

/**
 * @brief Get the Distance Between Points a and b
 *
 * @param a
 * @param b
 * @return double
 */

double SimulationPickandPlace::getDistanceBetweenPoints(ignition::math::Vector3d a, ignition::math::Vector3d b) {
    double x_diff = a[0] - b[0];
    double y_diff = a[1] - b[1];
    double z_diff = a[2] - b[2];
    double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2) + std::pow(z_diff, 2));
    return distance;
}
