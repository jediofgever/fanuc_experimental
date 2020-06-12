/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-27 10:16:06
 * @modify date 2020-02-27 10:16:06
 * @desc [description]
 */
#include <ros/ros.h>
#include <Eigen/Core>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math4/ignition/math/Vector3.hh>

#include <fanuc_arm_perception_utilities/pickplace/PickandPlacer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <fanuc_arm_perception_utilities/utils.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <std_msgs/Bool.h>

namespace gazebo {

/**
 * @brief This Class cooperates with pickplace/SimulationPickPlace in order to perform pick and place in gazebo simulator
 *
 */
class SimulationPickandPlace : public WorldPlugin {
   private:
    // for storing the pointer of this world
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // ROS node handler to handle usual ROS stuff(publisher , subscriber etc..)
    ros::NodeHandle *nh_;

    // Publish ground truth 3d box of each object for pick and place
    ros::Publisher gtBBX_pub_;

    ros::Publisher ros_pub_to_see_if_robot_is_in_scan_pose_pub_;

    // Listener for TF tree
    tf::TransformListener *listener_;

   public:
    /**
     * @brief Construct a new Simulation Pickand Place object
     *
     */
    SimulationPickandPlace();

    /**
     * @brief Destroy the Simulation Pickand Place object
     *
     */
    ~SimulationPickandPlace();

    /**
     * @brief return a rndom double rabging from fMin to fMax
     *
     * @param fMin
     * @param fMax
     * @return double
     */
    double fRand(double fMin, double fMax);

    /**
     * @brief Execute once on start of this plugin
     *
     * @param _parent
     */
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

    /**
     * @brief in each gazebo frame , this method is called
     *
     */
    void OnUpdate();

    /**
     * @brief Get the Distance Between Points a and b
     *
     * @param a
     * @param b
     * @return double
     */
    double getDistanceBetweenPoints(ignition::math::Vector3d a, ignition::math::Vector3d b);
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimulationPickandPlace);
}  // namespace gazebo