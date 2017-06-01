/*
 * motor_model.h
 *
 *  Created on: 04.05.2017
 *      Author: robot
 */

#ifndef HORUS_GAZEBO_PLUGINS_INCLUDE_PROPULSION_MODEL_H_
#define HORUS_GAZEBO_PLUGINS_INCLUDE_PROPULSION_MODEL_H_

#include <gazebo-7/gazebo/math/MathTypes.hh>
#include <gazebo-7/gazebo/math/Pose.hh>
#include <gazebo-7/gazebo/math/Matrix3.hh>
#include <gazebo-7/gazebo/math/Vector3.hh>

namespace Propulsion{

struct PropulsionParam {
		double hover_thrust; // gravitational forces
		double n_blades ; // number of blades
		double blade_radius ;
		//double disc_area ;
		double density_air;
		double l_chord_av;
//		double solidity;
		double lift_slope; // lift coefficient linearly dependent on AOA
		double C_drag; // airfoil drag coefficient at 70 % radial station
		double pitch_0;
		double pitch_twist; // linear coefficient of pitch function over r
		int dir;
		gazebo::math::Pose* rotor_pose;
};

struct PropulsionState {
		double my_advance; // advance ratio
		double lambda_inflow; // inflow ratio
		double v_ind;
		double F_thrust;
		double T_roll;
		double T_shaft;
		double F_hub;
		gazebo::math::Vector3* force_tot ;
		gazebo::math::Vector3* torque_tot ;
};

struct PropulsionInput {
		double omega ;
		gazebo::math::Vector3* angular ;
		gazebo::math::Vector3* vel ;
};

class PropulsionModel
{
public:
	PropulsionModel() ;
	virtual ~PropulsionModel() ;

	PropulsionParam GetPropulsionParam();
	int SetPropulsionParam(PropulsionParam &param);
	int SetPropulsionInput(PropulsionInput &input);
	int SetName(std::string name);
	std::string GetName() ;
	PropulsionState update(double dt);

private:

	PropulsionParam param_;
	PropulsionState state_;
	PropulsionInput input_;
	double solidity, disc_area;
	double c_omega2, c_thrust_my2, c_thrust_lambda2, c_thrust_0 ;
	double c_hub_my, c_hub_mylambda;
	double c_shaft_0, c_shaft_my2, c_shaft_lambda, c_shaft_lambda2;
	double c_roll_my , c_roll_mylambda ;

	std::string name_ ;

	bool base_as_ref;

	void UpdateConstants();
};

}

#endif /* HORUS_GAZEBO_PLUGINS_INCLUDE_PROPULSION_MODEL_H_ */


