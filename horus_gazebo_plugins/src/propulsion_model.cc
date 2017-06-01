/*
 * motor_model.cc
 *
 *  Created on: 04.05.2017
 *      Author: robot
 */

#include "propulsion_model.h"
#include <iostream>
#include <math.h>
namespace Propulsion
{

PropulsionModel::PropulsionModel()
{
	base_as_ref = true ;
	param_.C_drag = 0.6;
	param_.hover_thrust = 5*9.81 ;
	param_.blade_radius = 0.2;
	param_.density_air = 0.5;
	param_.dir = 1.0;
//	param_.disc_area = 0.2*0.2*M_PI ;
	param_.lift_slope = 0.1;
	param_.pitch_0 = 0.1 ;
	param_.pitch_twist = 0.1;
	param_.l_chord_av = 0.02;
	param_.rotor_pose = new gazebo::math::Pose(0.0,0.0,0.0,0.0,0.0,0.0);
//	param_.solidity = 1/10; // blade area/disc area
	UpdateConstants();

	state_.force_tot = new gazebo::math::Vector3;
	state_.torque_tot = new gazebo::math::Vector3;

	input_.angular = new gazebo::math::Vector3(0,0,0);
	input_.vel = new gazebo::math::Vector3(0,0,0);
}
PropulsionModel::~PropulsionModel()
{}

PropulsionParam PropulsionModel::GetPropulsionParam()
{
	return param_;
}

int PropulsionModel::SetPropulsionParam(PropulsionParam &param)
{
	param_ = Propulsion::PropulsionParam(param) ;
	UpdateConstants();
	return 1;
}

int PropulsionModel::SetPropulsionInput(PropulsionInput &input)
{
	input_ = input ;
	return 1;
}

int PropulsionModel::SetName(std::string name)
{
	this->name_ = name;
	return 1;
}

std::string PropulsionModel::GetName()
{
	return this->name_;
}

PropulsionState PropulsionModel::update(double dt)
{
	// Update equations by IFKM (respectively Stanford Micro Air Vehicle)
	// velocity of rotor coordinate system in body frame
	gazebo::math::Vector3 rotor_vel = *input_.vel ;
	gazebo::math::Matrix3 proj_xy = gazebo::math::Matrix3(1,0,0,0,1,0,0,0,0);

	if (base_as_ref)
	{
		rotor_vel += input_.angular->Cross(param_.rotor_pose->pos);
		// rotate velocity to rotor frame to get x-y-velocity
		rotor_vel = param_.rotor_pose->rot.RotateVector(rotor_vel);
	}
	// Use projection to get x-y-velocity
	gazebo::math::Vector3 vec_xy ;
	vec_xy = proj_xy*rotor_vel;

	double vel_xy = vec_xy.GetLength();
	// induced velocity as a function of x-y-velocity
	state_.v_ind = sqrt(-vel_xy*vel_xy/2 + sqrt(pow(vel_xy,4)/4 + pow(param_.hover_thrust/2/param_.density_air/disc_area,2)));
	// get z-velocity and devide by omega if not zero
	if ( abs(input_.omega) <= 10)
	{
		state_.lambda_inflow = 0;
		state_.my_advance = 0;
	}
	else
	{
		state_.lambda_inflow = (state_.v_ind - rotor_vel.z ) / (input_.omega * param_.blade_radius);
		// compute advance ratio
		state_.my_advance = vel_xy / (input_.omega * param_.blade_radius) ;
	}
	// compute polynomials
	// Thrust acts in z-direction
	state_.F_thrust = input_.omega*abs(input_.omega)*c_omega2*
			(c_thrust_0 + c_thrust_lambda2* pow(state_.lambda_inflow,2)+ c_thrust_my2 * pow(state_.my_advance,2));
	state_.force_tot->Set(0,0,state_.F_thrust);
	// Hub-force acts in line with x-y-velocity
	state_.F_hub = input_.omega*abs(input_.omega)*c_omega2* (c_hub_my*state_.my_advance + c_hub_mylambda*state_.my_advance * state_.lambda_inflow);
	*state_.force_tot += vec_xy.Normalize()*state_.F_hub;
	// shaft torque acts about z-axis
	state_.T_shaft = input_.omega*abs(input_.omega)*param_.blade_radius*c_omega2*(c_shaft_0 + c_shaft_lambda * state_.lambda_inflow +
			c_shaft_lambda2*pow(state_.lambda_inflow,2)+c_shaft_my2*pow(state_.my_advance,2));
	// shaft torque is positive (=> negate for propagation in motor model
	state_.torque_tot->Set(0,0,param_.dir*state_.T_shaft);
	// rolling torque acts about x-y-direction
	state_.T_roll = param_.dir*input_.omega*abs(input_.omega)*param_.blade_radius*c_omega2*(c_roll_my* state_.my_advance + c_roll_mylambda * state_.my_advance * state_.lambda_inflow);
	*state_.torque_tot += vec_xy.Normalize()*state_.T_roll;

	// body-center as reference:
	if(base_as_ref)
	{
		*state_.torque_tot = param_.rotor_pose->rot.RotateVectorReverse(*state_.torque_tot);
		*state_.force_tot = param_.rotor_pose->rot.RotateVectorReverse(*state_.force_tot);
		*state_.torque_tot += param_.rotor_pose->pos.Cross(*state_.force_tot);
		std::cout << "Prop State: \n" << "force: " <<
				*state_.force_tot << "\ntorque: " << *state_.torque_tot <<
				"\nlambda: " << state_.lambda_inflow <<
				"\nmy: " << state_.my_advance << "omega: " << input_.omega <<
				"\nv_ind: "<< state_.v_ind <<
				"\nv_xy: " << vec_xy*vel_xy << std::endl ;

	}

	// print state
//	std::cout << "Motor Status: \n" <<
//			"Pos: " << this->param_.rotor_pose->pos
//			<< "\nF_Thrust: " << state_.F_thrust
//			<< "\nF_Hub: " << state_.F_hub
//			<< "\nT_shaft: " << state_.T_shaft
//			<< "\nT_roll: " << state_.T_roll << std::endl ;

	return state_;
}

void PropulsionModel::UpdateConstants()
{
	disc_area = M_PI*param_.blade_radius* param_.blade_radius;
	solidity = param_.n_blades*param_.l_chord_av/(param_.blade_radius*M_PI) ;

	c_omega2 = solidity*param_.density_air*param_.lift_slope*disc_area*pow(param_.blade_radius,2);

	c_thrust_0 = (param_.pitch_0/6.0-param_.pitch_twist/8.0 ) ;
	c_thrust_my2 = (param_.pitch_0/4.0 - param_.pitch_twist/8.0);
	c_thrust_lambda2 = -1.0/4.0 ;

	c_hub_my = param_.C_drag / (4.0*param_.lift_slope);
	c_hub_mylambda = (param_.pitch_0-param_.pitch_twist/2.0)/4.0;

	c_shaft_0 = param_.C_drag/(8.0*param_.lift_slope) ;
	c_shaft_my2 = param_.C_drag/(8.0*param_.lift_slope) ;
	c_shaft_lambda = param_.pitch_0/6.0-param_.pitch_twist/8.0 ;
	c_shaft_lambda2 = -1.0/4.0;

	c_roll_my = -param_.pitch_0 / 6.0 + param_.pitch_twist/8.0;
	c_roll_mylambda = - 1.0/8.0;

	std::cout <<"Propulsion Model Parameters: \n"
			<< "c_omega2 = " << c_omega2 << "\n"
			<< "c_thrust_0 = " << c_thrust_0 << "\n"
			<< "c_thrust_my2 = " << c_thrust_my2 << "\n"
			<< "c_thrust_lambda2 = " << c_thrust_lambda2 << "\n"
			<< "c_hub_my = " << c_hub_my << "\n"
			<< "c_hub_mylambda = " << c_hub_mylambda << "\n"
			<< "c_shaft_0 = " << c_shaft_0 << "\n"
			<< "c_shaft_my2 = " << c_shaft_my2 << "\n"
			<< "c_shaft_lambda = " << c_shaft_lambda << "\n"
			<< "c_shaft_lambda2 = " << c_shaft_lambda2 << "\n"
			<< "c_roll_my2 = " << c_roll_my << "\n"
			<< "c_roll_mylambda = " << c_roll_mylambda << "\n"<< std::endl;
}

}

