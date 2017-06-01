/*
 * motor_model.h
 *
 *  Created on: 04.05.2017
 *      Author: robot
 */

#ifndef HORUS_GAZEBO_PLUGINS_INCLUDE_MOTOR_MODEL_H_
#define HORUS_GAZEBO_PLUGINS_INCLUDE_MOTOR_MODEL_H_

namespace Motor{

struct MotorParam {
		double inertia_motor ;
		double inertia_rotor ;
		double resistance;
		double inductance;
		double k_motor;
		double transmission;
		double efficiency;
		double k_fric;
};

struct MotorState {
		double omega_motor;
		double current;
		double omega_rotor;
};

struct MotorInput {
		double  voltage;
		double  torque;
};

class MotorModel
{
public:
	MotorModel() ;
	virtual ~MotorModel() ;

	MotorParam GetMotorParam();
	int SetMotorParam(MotorParam &param);
	int SetMotorInput(MotorInput &input);
	MotorState update(double dt);
	MotorState GetMotorState();

private:

	MotorParam param_;
	MotorState state_;
	MotorInput input_;
	double A,B,C ,den;
	void UpdateConstants();
};

}

#endif /* HORUS_GAZEBO_PLUGINS_INCLUDE_MOTOR_MODEL_H_ */
