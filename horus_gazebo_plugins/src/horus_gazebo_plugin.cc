#ifndef _HORUS_GAZEBO_PLUGIN_HH_
#define _HORUS_GAZEBO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "dynamic_reconfigure/server.h"
#include "horus_gazebo_plugins/control_drConfig.h"

#include "motor_model.h"
#include "propulsion_model.h"

using namespace gazebo ;

/// \brief A plugin to control a Velodyne sensor.
class HorusPlugin : public ModelPlugin
{
	/// \brief Constructor
public: HorusPlugin() {}

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	control_method = 1;
	// TEST of Motor Model Class
	motor_model_ = new Motor::MotorModel;
	motor_param_ = motor_model_->GetMotorParam();
	std::cout << "Induktivitaet:" << motor_param_.inductance << std::endl ;

	motor_param_.k_motor = 0.05 ;
	motor_param_.resistance = 0.05 ;
	motor_param_.inertia_rotor = 0.008 ;
	motor_param_.inertia_motor = 0.004 ;

	base_as_ref = true;

	// Test of Propulsion Model Class
	propulsion_model_ = new Propulsion::PropulsionModel ;
	propulsion_param_ = propulsion_model_->GetPropulsionParam();
	propulsion_param_.C_drag = 0.0005;
	propulsion_param_.rotor_pose = new math::Pose(1,1,0.5,0,0,0);
	propulsion_param_.pitch_0 = 0.7 ;
	propulsion_param_.pitch_twist = 0.5 ;
	propulsion_param_.n_blades = 2;
	propulsion_param_.l_chord_av = 0.02 ;
	propulsion_param_.hover_thrust = 9.81*4;
	propulsion_param_.lift_slope = 10 ;
	propulsion_param_.blade_radius = 0.2;
	propulsion_model_ = new Propulsion::PropulsionModel ;
	propulsion_model_->SetPropulsionParam(propulsion_param_) ;

	double velocity = 0;
	// Just output a message for now
	std::cerr << "\nThe push plugin is attached to model[" <<
			_model->GetName() << "]\n";
	// Check that the velocity element exists, then read the value
	base_link_name = std::string("base_link");
	if (_sdf->HasElement("base_frame_name"))
		base_link_name = _sdf->Get<std::string>("base_frame_name");

	this->p_roll_= 100;
	this->p_pitch_ = 100;
	this->p_yaw_ = 100;
	this->p_z_ = 100;

	// Safety check
//	if (_model->GetJointCount() == 0)
//	{
//		std::cerr << "Invalid joint count, Push plugin not loaded\n";
//		return;
//	}

	// Store the model pointer for convenience.
	this->model = _model;

	t_prev= this->model->GetWorld()->GetSimTime();

	// assumption of link name
	this->base_link = _model->GetLink(base_link_name.c_str());

	// Create the node
	this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
	this->node->Init(this->model->GetWorld()->GetName());
#else
	this->node->Init(this->model->GetWorld()->Name());
#endif

	// Create a topic name
	std::string topicName = "~/" + this->model->GetName() + "/wrench_cmd";

	// Subscribe to the topic, and register a callback
	this->sub = this->node->Subscribe(topicName,
			&HorusPlugin::OnMsg, this);

	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",
				ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	ros::NodeHandle* node_handle = new ros::NodeHandle("gazebo_client");
	this->rosNode.reset(node_handle);

	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
			ros::SubscribeOptions::create<geometry_msgs::Wrench>(
					"/" + this->model->GetName() + "/torque_cmd",
					1,
					boost::bind(&HorusPlugin::OnRosMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	ros::SubscribeOptions so_vel =
			ros::SubscribeOptions::create<geometry_msgs::Twist>(
					"/" + this->model->GetName() + "/twist_cmd",
					1,
					boost::bind(&HorusPlugin::OnRosVelMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);
	this->rosSubVel = this->rosNode->subscribe(so_vel);

	// Start ROS-Publisher
	this->rosPubMotorSpeed = rosNode->advertise<std_msgs::Float32MultiArray>(
							"/"+this->model->GetName() + "/motorspeed",5);
	this->rosPubWrench = rosNode->advertise<geometry_msgs::Wrench>("/"+this->model->GetName() + "/wrench",10);

	// Spin up the queue helper thread.
	this->rosQueueThread =
			std::thread(std::bind(&HorusPlugin::QueueThread, this));

	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&HorusPlugin::OnUpdate,this));

	physics::Link_V child_links = this->base_link->GetChildJointsLinks();
	int childcount = this->base_link->GetChildCount();
	std::cout << "no  of childs: " << childcount << std::endl ;
	for (int i = 0 ; i < childcount ; i++)
	{
		physics::BasePtr bla = this->base_link->GetChild(i);
		std::cout << bla->GetName() << std::endl;

	}

	physics::Collision_V collisions = this->base_link->GetCollisions();
	for (int i = 0 ; i< collisions.size() ; i++)
	{
		if (collisions[i]->GetName().find(std::string("rotor")) != std::string::npos )
		{
			math::Pose coll_pose = collisions[i]->GetWorldPose();
			std::cout <<  collisions[i]->GetName() << "xyz : " <<
			coll_pose.pos.x << " " << coll_pose.pos.y << " "<<  coll_pose.pos.z << std::endl;
		}
	}

	for (int i = 0 ; i < child_links.size() ; i++)
	{
		if (child_links[i]->GetName().find(std::string("rotor")) != std::string::npos )
		{
			rotor_links.push_back(child_links[i]);
			std::cout << "child link " << i << ":  "<< child_links[i]->GetName().c_str() <<"\n";
			Propulsion::PropulsionModel* prop_ptr = new Propulsion::PropulsionModel ;
			propulsion_param_.rotor_pose = new math::Pose(child_links[i]->GetInitialRelativePose());

			std::cout << "pos: " << child_links[i]->GetInitialRelativePose().pos <<"\n";

			//TODO: Read direction and mixing-table from yaml
			mixer_thrust.push_back(1.0);
			if(child_links[i]->GetName().find(std::string("rear"))!= std::string::npos)
				mixer_pitch.push_back(1.0);
			else mixer_pitch.push_back(-1.0);
			if(child_links[i]->GetName().find(std::string("right"))!= std::string::npos)
				mixer_roll.push_back(-1.0);
			else mixer_roll.push_back(1.0);
			if(child_links[i]->GetName().find(std::string("right"))!= std::string::npos &&
					child_links[i]->GetName().find(std::string("rear"))!= std::string::npos ||
				child_links[i]->GetName().find(std::string("left"))!= std::string::npos &&
					child_links[i]->GetName().find(std::string("front"))!= std::string::npos )
			{
				mixer_yaw.push_back(-1.0);
				propulsion_param_.dir = -1;
			}
			else
			{
				mixer_yaw.push_back(1.0);
				propulsion_param_.dir = 1;
			}

			prop_ptr->SetPropulsionParam(propulsion_param_);
			prop_ptr->SetName(child_links[i]->GetName());
			propulsion_ptrs.push_back(prop_ptr);

			Motor::MotorModel* motor_ptr = new Motor::MotorModel ;
			motor_ptr->SetMotorParam(motor_param_);
			motor_ptrs.push_back(motor_ptr);

			std::cout << "Configuration for RotorLink " << child_links[i]->GetName()
					<< "\n" << "thrust \t yaw \t roll \t pitch \n " <<
					mixer_thrust.back() <<"\t " << mixer_yaw.back() << "\t" << mixer_roll.back() << "\t"
					<< mixer_pitch.back() << std::endl ;

		}
	}

	//f = boost::bind(&HorusPlugin::dynamicReconfigureCallback , this,_1, _2);
	this->server.reset(new dynamic_reconfigure::Server<horus_gazebo_plugins::control_drConfig>(*node_handle));
	this->server->setCallback(boost::bind(&HorusPlugin::dynamicReconfigureCallback , this,_1, _2));
}

/// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity
//public: void SetVelocity(const double &_vel)
//{
//	// Set the joint's target velocity.
//	this->model->GetJointController()->SetVelocityTarget(
//			this->joint->GetScopedName(), _vel);
//}

public: void SetWrench(const gazebo::math::Vector3 &force, const gazebo::math::Vector3 &torque)
{

	std::cout <<"setting wrench:"<< std::endl;
	this->base_link->AddRelativeForce(force);
//	std::cout << "set force" << std::endl;
	this->base_link->AddRelativeTorque(torque);
//	std::cout << "set torque" << std::endl ;
//	base_link->Update();
//	std::cout << "update" << std::endl;
}

protected: void OnUpdate()
{
    common::Time t_now = this->model->GetWorld()->GetSimTime();
    common::Time dt = t_now - t_prev ;
    std::cout << "dt = " << dt.Double() << std::endl;

	// simple Proportional attitude controller
	math::Pose world_pose = this->base_link->GetWorldInertialPose();
	math::Vector3 body_vel = this->base_link->GetRelativeLinearVel();
	math::Vector3 body_ang_vel = this->base_link->GetRelativeAngularVel();

	std::cout << "World Pose: \n" << world_pose << std::endl;
	std::cout << "vel: \n" << body_vel << body_ang_vel << std::endl;

//    double ang_vel_x = p_roll_*(this->Pose_cmd_.rot.GetRoll() - world_pose.rot.GetRoll());
//    double ang_vel_y = p_pitch_*(this->Pose_cmd_.rot.GetPitch() - world_pose.rot.GetPitch());
//    double torque_x = d_roll_*(ang_vel_x - body_vel.x);
//    double torque_y = d_pitch_*(ang_vel_y - body_vel.y);
//    double torque_z = p_yaw_*(this->Pose_cmd_.rot.GetYaw() - body_ang_vel.z);
//    double force_z = p_z_*(this->Pose_cmd_.pos.z - body_vel.z ) ;
	double ang_vel_x,ang_vel_y, torque_x, torque_y, torque_z, force_z ;

	// TODO: Implement PID-Controller as Class
	switch(control_method)
	{
		case 1:
		    ang_vel_x =p_roll_*Pose_cmd_.rot.GetRoll(); //p_roll_*(this->Pose_cmd_.rot.GetRoll() - world_pose.rot.GetRoll());
			ang_vel_y =p_pitch_*Pose_cmd_.rot.GetPitch(); //p_pitch_*(this->Pose_cmd_.rot.GetPitch() - world_pose.rot.GetPitch());
			torque_x = ang_vel_x ; //d_roll_*(ang_vel_x - body_vel.x);
			torque_y = ang_vel_y ;//d_pitch_*(ang_vel_y - body_vel.y);
			torque_z = p_yaw_*Pose_cmd_.rot.GetYaw() ;//p_yaw_*(this->Pose_cmd_.rot.GetYaw() - body_ang_vel.z);
			force_z = p_z_*(this->Pose_cmd_.pos.z  ) ;
			break;
		case 2:
		    ang_vel_x =p_roll_*Pose_cmd_.rot.GetRoll(); //p_roll_*(this->Pose_cmd_.rot.GetRoll() - world_pose.rot.GetRoll());
			ang_vel_y =p_pitch_*Pose_cmd_.rot.GetPitch(); //p_pitch_*(this->Pose_cmd_.rot.GetPitch() - world_pose.rot.GetPitch());
			torque_x = ang_vel_x ; //d_roll_*(ang_vel_x - body_ang_vel.x);
			torque_y = ang_vel_y ; //d_pitch_*(ang_vel_y - body_ang_vel.y);
			torque_z = p_yaw_*(this->Pose_cmd_.rot.GetYaw() - body_ang_vel.z);
			force_z = p_z_*(this->Pose_cmd_.pos.z  ) ;
			break;
		case 3:
			ang_vel_x =Pose_cmd_.rot.GetRoll(); //p_roll_*(this->Pose_cmd_.rot.GetRoll() - world_pose.rot.GetRoll());
			ang_vel_y =Pose_cmd_.rot.GetPitch(); //p_pitch_*(this->Pose_cmd_.rot.GetPitch() - world_pose.rot.GetPitch());
			torque_x = p_roll_*(ang_vel_x - body_ang_vel.x);
			torque_y = p_pitch_*(ang_vel_y - body_ang_vel.y);
			torque_z = p_yaw_*(this->Pose_cmd_.rot.GetYaw() - body_ang_vel.z);
			force_z = p_z_*(this->Pose_cmd_.pos.z  ) ;
			break;
		default:
			ang_vel_x =p_roll_*(this->Pose_cmd_.rot.GetRoll() - world_pose.rot.GetRoll());
			ang_vel_y =p_pitch_*(this->Pose_cmd_.rot.GetPitch() - world_pose.rot.GetPitch());
			torque_x = d_roll_*(ang_vel_x - body_ang_vel.x);
			torque_y = d_pitch_*(ang_vel_y - body_ang_vel.y);
			torque_z = p_yaw_*(this->Pose_cmd_.rot.GetYaw() - body_ang_vel.z);
			force_z = p_z_*(this->Pose_cmd_.pos.z  ) ;
	}

	std::cout <<"controller output: \n" <<
			"rpy: " << torque_x << " " <<
			torque_y << " " <<
			torque_z << "\n" << "z: " <<
			force_z << " " <<  std::endl;

    geometry_msgs::Wrench wrench_msg;
    std_msgs::Float32MultiArray omegas_msg;

    this->force_ = math::Vector3(0.0,0.0,0);
    this->torque_ = math::Vector3(0.0, 0.0, 0.0) ;

    math::Vector3 base_vel_lin(base_link->GetRelativeLinearVel());
    math::Vector3 base_vel_ang(base_link->GetRelativeAngularVel());

    // Evaluation of Motor and Propulsion Model
    for (int i = 0 ; i < rotor_links.size(); i++)
    {
    	std::cout <<"\n >> iteration " << i << " <<"<<  std::endl;
    	std::cout <<"mixer(r,p,y,z): " << mixer_roll[i] << " "
    			<< mixer_pitch[i] << " "
				<< mixer_yaw[i] << " "
				<< mixer_thrust[i] << " "<<  std::endl;

    	// generate input voltage from controller output
	double voltage_cmd = mixer_roll[i]*torque_x+
			mixer_pitch[i]*torque_y+
			mixer_thrust[i]*force_z ;

	// yaw control
	if(torque_z >= 0)
		voltage_cmd += mixer_yaw[i]*std::min(abs(torque_z*voltage_cmd),abs(voltage_cmd));
	else
		voltage_cmd += -mixer_yaw[i]*std::min(abs(torque_z*voltage_cmd),abs(voltage_cmd));

	std::cout <<"voltage: " << voltage_cmd <<  std::endl;

    Propulsion::PropulsionInput prop_input;
    prop_input.omega = this->motor_ptrs[i]->GetMotorState().omega_rotor;
    if (!base_as_ref)
    {
    	prop_input.vel = new math::Vector3(rotor_links[i]->GetRelativeLinearVel());
    	prop_input.angular = new math::Vector3(rotor_links[i]->GetRelativeAngularVel());
    }
    else
    {
    	prop_input.vel = &base_vel_lin ;
    	prop_input.angular = &base_vel_ang ;
    }
    this->propulsion_ptrs[i]->SetPropulsionInput(prop_input);
    Propulsion::PropulsionState prop_state = this->propulsion_ptrs[i]->update(dt.Double());
    std::cout << "Propulsion State "<< i <<" : \n"
    		<< "Thrust:  " << prop_state.F_thrust << "\n"
			<< "Hub:  "	<< prop_state.F_hub << "\n"
			<< "Yaw-Moment: " << -prop_state.T_shaft << "\n"
			<< "Roll-Moment: " << prop_state.T_roll << "\n"
			<< std::endl;

    this->force_ = this->force_ + *prop_state.force_tot ;
    this->torque_ = this->torque_ + *prop_state.torque_tot ;

    Motor::MotorInput motor_input ;
	motor_input.torque = -prop_state.T_shaft ;
    motor_input.voltage = voltage_cmd ;
    motor_ptrs[i]->SetMotorInput(motor_input);
    Motor::MotorState state = motor_ptrs[i]->update(dt.Double());

    std::cout << "Motor State: \nI_a = " << state.current <<
    		"\nw_m = " << state.omega_motor <<
			"\nw_r = " << state.omega_rotor << std::endl ;

    omegas_msg.data.push_back(state.omega_rotor);

    //TODO: option to change force reference points/links
    	// apply single force to specific links
    	if (!base_as_ref)
    	{
    	this->rotor_links[i]->AddRelativeForce(*prop_state.force_tot);
    	//	std::cout << "set force" << std::endl;
		this->rotor_links[i]->AddRelativeTorque(*prop_state.torque_tot);
    	}
    }
    std::cout <<"endfor"<< std::endl;

    // apply sum of forces to body_link
    if(base_as_ref)
    	this->SetWrench(this->force_,this->torque_);

    std::cout << "Force"<<": "	<< this->force_	<< std::endl;
    std::cout << "Torque"<<": "	<< this->torque_	<< std::endl;


    wrench_msg.force.x = force_.x;
	wrench_msg.force.y = force_.y;
	wrench_msg.force.z = force_.z;
    wrench_msg.torque.x = torque_.x;
	wrench_msg.torque.y = torque_.y;
	wrench_msg.torque.z = torque_.z;

	rosPubWrench.publish(wrench_msg);
	rosPubMotorSpeed.publish(omegas_msg);

	t_prev = t_now ;

}

/// \brief Handle incoming message
/// \param[in] _msg Repurpose a vector3 message. This function will
/// only use the x component.
private: void OnMsg(ConstVector3dPtr &_msg)
{
	//this->SetVelocity(_msg->x());
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public: void OnRosMsg(const geometry_msgs::WrenchConstPtr &_msg)
{
	this->force_ = gazebo::math::Vector3(_msg->force.x,_msg->force.y,_msg->force.z);
	this->torque_ = gazebo::math::Vector3(_msg->torque.x,_msg->torque.y,_msg->torque.z);
}

public: void OnRosVelMsg(const geometry_msgs::TwistConstPtr &_msg)
{
	Pose_cmd_ = math::Pose(math::Vector3(_msg->linear.x,_msg->linear.y,_msg->linear.z),
			math::Quaternion(math::Vector3(_msg->angular.x,_msg->angular.y,_msg->angular.z)));

}

public: void dynamicReconfigureCallback(horus_gazebo_plugins::control_drConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: P_z = %f P_roll = %f P_pitch = %f P_yaw = %f",config.p_z,config.p_yaw,config.p_roll,config.p_pitch);
	this->p_z_ = config.p_z ;
	this->p_roll_ = config.p_roll ;
	this->p_pitch_ = config.p_pitch ;
	this->p_yaw_ = config.p_yaw ;
	this->d_roll_ = config.d_roll ;
	this->d_pitch_ = config.d_pitch ;


	for (int i = 0 ; i < this->rotor_links.size(); i++)
	{

		propulsion_param_ = propulsion_ptrs[i]->GetPropulsionParam();
		motor_param_ = motor_ptrs[i]->GetMotorParam();

		this->propulsion_param_.C_drag = config.c_drag_mean;
		this->propulsion_param_.lift_slope = config.a_lift;
		this->propulsion_param_.pitch_0 = config.pitch_0;
		this->propulsion_param_.pitch_twist = config.pitch_tw;
		this->propulsion_param_.hover_thrust = config.hover_thrust / (double)rotor_links.size() ;

		this->motor_param_.k_motor = config.k_motor;
		this->motor_param_.inertia_motor = config.J_motor;
		this->motor_param_.resistance = config.R_motor;

		this->control_method = config.control_method;

		propulsion_ptrs[i]->SetPropulsionParam(propulsion_param_);
		motor_ptrs[i]->SetMotorParam(motor_param_);
	}
}


/// \brief ROS helper function that processes messages
private: void QueueThread()
{
	static const double timeout = 0.01;
	while (this->rosNode->ok())
	{
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}


/// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief Pointer to the joint.
private: physics::JointPtr joint;

private: physics::LinkPtr base_link;

private: std::string base_link_name;

/// \brief A PID controller for the joint.
private: common::PID pid;

/// \brief A node used for transport
private: transport::NodePtr node;

/// \brief A subscriber to a named topic.
private: transport::SubscriberPtr sub;

/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;
private: ros::Subscriber rosSubVel;

/// \breif A ROS Publisher
private: ros::Publisher rosPubWrench;
private: ros::Publisher rosPubMotorSpeed;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
private: gazebo::math::Vector3 force_ ;
private: gazebo::math::Vector3 torque_ ;
private: gazebo::math::Pose Pose_cmd_;

private: common::Time t_prev ;

private: boost::shared_ptr<dynamic_reconfigure::Server<horus_gazebo_plugins::control_drConfig> > server;
private: dynamic_reconfigure::Server<horus_gazebo_plugins::control_drConfig>::CallbackType f;

private: double p_roll_,d_roll_,d_pitch_, p_yaw_, p_pitch_, p_z_;

private: event::ConnectionPtr update_connection_;

private: Motor::MotorParam motor_param_;
private: Motor::MotorModel* motor_model_;

private: Propulsion::PropulsionModel* propulsion_model_;
private: Propulsion::PropulsionParam propulsion_param_;

private: std::vector<Propulsion::PropulsionModel*> propulsion_ptrs ;
private: std::vector<Motor::MotorModel*> motor_ptrs ;

private: physics::Link_V rotor_links;

private: std::vector<float> mixer_thrust ;
		 std::vector<float> mixer_roll ;
		 std::vector<float> mixer_yaw ;
		 std::vector<float> mixer_pitch ;

private: int control_method;

private: bool base_as_ref;

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(HorusPlugin)

#endif
