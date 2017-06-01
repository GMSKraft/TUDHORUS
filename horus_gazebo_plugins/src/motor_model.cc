/*
 * motor_model.cc
 *
 *  Created on: 04.05.2017
 *      Author: robot
 */

#include "motor_model.h"
#include <iostream>
namespace Motor
{

MotorModel::MotorModel()
{
	param_.inductance = 1;
	param_.inertia_motor = 0.03;

	param_.resistance = 0.2;
	param_.k_motor = 1 ;
	param_.transmission = 1 ;
	param_.efficiency = 1 ;

	param_.k_fric = 0.0001 ;

	state_.current = 0;
	state_.omega_motor = 0;
	state_.omega_rotor = 0;
	UpdateConstants();
}
MotorModel::~MotorModel()
{}

MotorParam MotorModel::GetMotorParam()
{
	return param_;
}

int MotorModel::SetMotorParam(MotorParam &param)
{
	param_ = param ;
	UpdateConstants();
	return 1;
}

int MotorModel::SetMotorInput(MotorInput &input)
{
	input_ = input ;
	return 1;
}

MotorState MotorModel::GetMotorState()
{
	return state_;
}

MotorState MotorModel::update(double dt)
{
	double u_a = input_.voltage ;
	state_.omega_rotor = state_.omega_rotor
			+ dt * (C*u_a - A*state_.omega_rotor - B*abs(state_.omega_rotor)*state_.omega_rotor - input_.torque/den);//- input_.torque/den

	state_.omega_motor = state_.omega_rotor / param_.transmission / param_.transmission;

	state_.current = (input_.voltage-state_.omega_motor * param_.k_motor)/param_.resistance;

	return state_;
}

void MotorModel::UpdateConstants()
{
	den = param_.inertia_motor
			+ param_.inertia_rotor/(param_.efficiency* param_.transmission* param_.transmission);

	A = param_.k_motor *param_.k_motor * param_.transmission / param_.resistance / den;
	B = param_.k_fric* param_.transmission*param_.transmission  / den;
	C = param_.k_motor / param_.resistance / den;
	std::cout <<"A= "<< A << " B=" << B << " C=" << C << std::endl;
}

}

