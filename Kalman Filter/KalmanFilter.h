// Kalman Filter.h : Include file for standard system include files,
// or project specific include files.

#pragma once


#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#define Kalman_1d 0
#define Kalman_md 1

class Gaussian
{
public:

	
	Gaussian();
	Gaussian(double mean , double variance);


	void SetMean(double mean);

	void SetVar(double var);

	double GetMean(void);

	double GetVar(void);

private :
	double mean;  //Mean of the gaussian
	double variance; //Var of the gaussian
};

///Multiplication of Gaussian is different from normal multiblication
Gaussian MultiplyGaussian( Gaussian & x,  Gaussian  & y);


Gaussian AddGaussian( Gaussian  & x,  Gaussian  & y);



//Kalman Class
///It automaticly creates a 3 Gaussian Classes , one for measurment , one for belief and one for Motion
///so we need to make it a template class , to allow the user enter the datatype of Kalman\ 
///which internaly reflects in Gaussian
// TODO: Reference additional headers your program requires here.

class Kalman
{
public:
	Kalman();


	//initial belief
	Kalman(Gaussian Belief);



	Gaussian MeasurmentUpdate()
	{
		Gaussian Result;
		Result = MultiplyGaussian(this->Belief, this->Measurment);
		this->Belief = Result;
		return Result;
	}
	Gaussian MotionUpdate()
	{
		Gaussian Result;
		Result = AddGaussian(this->Belief, this->Motion);
		this->Belief = Result;
		return Result;
	}

	Gaussian FinalEstimation()
	{
		return Belief;
	}

	void NewMeasurment(Gaussian NM);
	void NewMotion(Gaussian NM);


	~Kalman();

private:
	Gaussian Belief;
	Gaussian Measurment;
	Gaussian Motion;
};




//Mutli Kalman

class KalmanNDimentional
{
public:
	KalmanNDimentional();
	KalmanNDimentional(int d , int number_of_measurment);
	~KalmanNDimentional();

	void SetState(Eigen::MatrixXd X);
	void SetP(Eigen::MatrixXd P);
	void SetU(Eigen::MatrixXd U);
	void SetF(Eigen::MatrixXd F);
	void SetH(Eigen::MatrixXd H);
	void SetR(Eigen::MatrixXd R);
	void SetI(Eigen::MatrixXd I);

	Eigen::MatrixXd GetState() {return this->X; }
	Eigen::MatrixXd GetP() { return this->P; }
	Eigen::MatrixXd GetU() { return this->U; }
	Eigen::MatrixXd GetF() { return this->F; }
	Eigen::MatrixXd GetH() { return this->H; }
	Eigen::MatrixXd GetR() { return this->R; }
	Eigen::MatrixXd GetZ() { return this->Z; }
	Eigen::MatrixXd GetY() { return this->Y; }
	Eigen::MatrixXd GetS() { return this->S; }
	Eigen::MatrixXd GetK() { return this->K; }


	void MeasurmentUpdate();
	void MotionUpdate();

	void NewMeasurment(Eigen::MatrixXd Me);
	void NewMotion(Eigen::MatrixXd Mo);

	Eigen::MatrixXd FinalEstimation()
	{
		return this->X;
	}

private:
	Eigen::MatrixXd X;  ///State of the system
	Eigen::MatrixXd P;  ///proccess covariance matrix , initial uncertainity
	Eigen::MatrixXd U;  ///External motion 
	Eigen::MatrixXd F;  ///Next state function , same as A matrix , it just naming
	Eigen::MatrixXd H;  ///Measurment function 
	Eigen::MatrixXd R;  ///Measurment Uncertainity
	Eigen::MatrixXd I;  ///Identity matrix
	
	Eigen::MatrixXd Z; ///new measurment taken 
	Eigen::MatrixXd Y; ///Erro btwn measurment and state
	Eigen::MatrixXd K; ///Kalman Gain

	Eigen::MatrixXd S; ///helper matrix 
};
