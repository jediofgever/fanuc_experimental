#include <ros/ros.h>
#include <Eigen/Core>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math4/ignition/math/Vector3.hh>

#include <arm_perception_utilities/pickplace/PickandPlacer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <arm_perception_utilities/utils.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace gazebo {

class SimulationPickandPlace : public WorldPlugin {
    // Pointer to the model
   private:
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    PickandPlacer *pickplacer_;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;
    bool flag = true;

    ros::Publisher objects_tobe_picked_pub_;
    ros::NodeHandle *nh_;
    ros::Publisher gtBBX_pub_;
    tf::TransformListener *listener_;

   public:
    SimulationPickandPlace();
    ~SimulationPickandPlace();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate();
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimulationPickandPlace);
}  // namespace gazebo