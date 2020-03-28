

#include "KalmanFilter.h"
int main()
{
#if Kalman_1d
	std::cout << "Testing Using multi 1d kalman filter ........... " << std::endl;
	std::vector<double> Measurments = { 5.0,6.0,7.0,9.0,10.0 };
	std::vector<double> Motions = { 1.0,1.0,2.0,1.0,1.0 };

	double MeasurmentSigma = 4.0;
	double MotionSigma = 2.0;
	double InitMean = 0.0; //start @ location 0 in 1d 
	double InitVar = 1000.0; //Iam not certain at all that iam in location 0
	int i = 0;
	Gaussian Belief(InitMean, InitVar);

	Kalman K(Belief);

	Gaussian Measurment(0,0), Motion(0,0); //Create Measurment an Motion Gaussians
	//Loop till the data finished 
	while (i<Measurments.size())
	{
		Measurment.SetVar(MeasurmentSigma);
		Measurment.SetMean(Measurments.at(i));

		//Initial i was in some location , i take a measurment from sensor 
		K.NewMeasurment(Measurment);
		///so i need to update my belief 
		K.MeasurmentUpdate();


		///then i moved with car , so i need to add new motion to my kalman
		Motion.SetVar(MotionSigma);
		Motion.SetMean(Motions.at(i));
		K.NewMotion(Motion);

		///after i moved i need to update my belief again
		K.MotionUpdate();

		i++;
	}

	std::cout << "The final estimation using 1d kalman filter is  " << K.FinalEstimation().GetMean() << std::endl;
#endif
	//Kalman filter for multi dimention 

#if Kalman_md
	std::cout << "Testing Using multi variant kalman filter ........... " << std::endl;

	///First create a class of multi kalaman and pass to it the dimention you need
	///and the number of measurment your sensor calculate which by default  1 
	///we will use single measurment @ a time that represents the position 
	///and use 2 states represent position and velocity 
	int dimention = 2, measurment = 1,i=0; 
	KalmanNDimentional k_md(dimention, measurment);
	std::vector<double> MEASURMENT = { 1,2,3 };
	///Creationg our matrices 
	Eigen::MatrixXd X(dimention, 1);
	Eigen::MatrixXd P(dimention, dimention);
	Eigen::MatrixXd U(dimention, 1);
	Eigen::MatrixXd F(dimention, dimention);
	Eigen::MatrixXd H(measurment, dimention);
	Eigen::MatrixXd R(measurment, 1);
	Eigen::MatrixXd Z(measurment, 1);  ///measurment matrix
	Eigen::MatrixXd I(dimention, dimention);
	
	X << 0.0,
		 0.0;   ///initial state

	std::cout << "State initialy is : " << std::endl << X << std::endl;

	///we are not certian about the position and the velocity 
	///they are uncorrelated till now
	P <<1000.0,0.0,
		0.0,1000.0;   ///initial uncertainty 


	std::cout << "P initialy is : " << std::endl << P << std::endl;

	U << 0.0,
		0.0;   ///Discard motion

	std::cout << "Motion is discarded : " << std::endl << U << std::endl;


	F << 1.0, 1.0,
		0.0, 1.0;

	std::cout << "State Transition Matrix is : " << std::endl << F << std::endl;


	H << 1.0, 0.0;

	std::cout << "Measurment Transition Matrix is : " << std::endl << H << std::endl;


	R << 1.0;

	std::cout << "Noise in measurment  is : " << std::endl << R << std::endl;


	I << 1.0, 0.0,
		0.0, 1.0;

	std::cout << "Identity matrix  is : " << std::endl << I << std::endl;


	///Fill kalman that you have created with right matrices

	k_md.SetState(X);
	k_md.SetP(P);
	k_md.SetF(F);
	k_md.SetH(H);
	k_md.SetR(R);
	k_md.SetU(U);
	k_md.SetI(I);
	///here's the core of kalman filter 	

	Eigen::MatrixXd Y, S, KG;
	while (i < MEASURMENT.size())
	{
		Z << MEASURMENT.at(i);
		std::cout << "Taking measurment during my current location and " << std::endl;
		std::cout << "Update the new belief about my location and velocity " << std::endl;
		k_md.NewMeasurment(Z);

		///New Measurment 
		k_md.MeasurmentUpdate();

		std::cout << "Moving and lose some information about my state " << std::endl;
		///New motion which is always 0
		k_md.NewMotion(U);
		k_md.MotionUpdate();
		i++;
	}

	std::cout << "The final estimation of the position and velocity respectively using Multi-dimentional kalman filter is  " << std::endl;
	std::cout << k_md.FinalEstimation() << std::endl;
#endif
	return 0;
}
