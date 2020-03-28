// Kalman Filter.cpp : Defines the entry point for the application.
//

#include "KalmanFilter.h"


//Gaussian Class
///initializing gaussian with zero mean and variance

Gaussian::Gaussian():mean(0.0),variance(0.0) {};

///initializing Gaussian with custom mean and variance 

Gaussian::Gaussian(double mean,double variance):mean(mean) , variance(variance) {};



void::Gaussian::SetMean(double m) {

	this->mean = m;
}


void::Gaussian::SetVar(double var) {

	this->variance = var;
}


double::Gaussian::GetMean() {

	return this->mean;
}


double::Gaussian::GetVar() {

	return this->variance;
}



Gaussian MultiplyGaussian(Gaussian& x, Gaussian& y)
{
	Gaussian Result;

	Result.SetMean( (double) ( (x.GetVar() * y.GetMean()) + (y.GetVar() * x.GetMean()) ) / (x.GetVar() + y.GetVar()));

	Result.SetVar ( 1.0 / ((1.0 / x.GetVar()) + (1.0 / y.GetVar())) );

	return Result;
}

Gaussian AddGaussian(Gaussian& x,Gaussian& y)
{
	Gaussian Result;
	Result.SetMean(x.GetMean() + y.GetMean());

	Result.SetVar(x.GetVar() + y.GetVar());

	
	return Result;
}

//Kalman Class

Kalman::Kalman():Belief(),Measurment(),Motion(){}
Kalman::Kalman(Gaussian Init):Belief(Init){}

void::Kalman::NewMeasurment(Gaussian NM)
{
	this->Measurment = NM;
}
void::Kalman::NewMotion(Gaussian NM)
{
	this->Motion = NM;
}

Kalman::~Kalman()
{
}


KalmanNDimentional::KalmanNDimentional():X(),P(),U(),F(),H(),R(),I(),Z(),Y(),K(),S()
{}
KalmanNDimentional::KalmanNDimentional(int d,int number_of_measurment=1)
{
	//Follow this Convention 
	///Type the equation Which every matrix envolved to calculate the dimention correctly

	/// Eqn : X' = F*X + U 
	this->X = Eigen::MatrixXd::Zero(d, 1);
	this->U = Eigen::MatrixXd::Zero(d, 1);   ///External motion is same dimention as state

	///the cols of this matrix = the rows of state matrix 
	///and the rows of this matrix = the rows of External motion matrix 
	this->F = Eigen::MatrixXd::Zero(d,d);  ///State transition matrix

		/// Eqn : P' = F*P* F-transpose
	this->P = Eigen::MatrixXd::Zero(d, d);   ///initial uncertainity in state 

	///this H convert State to measurment 
	///this matrix rows depents on how many measurment we take 
	///if we take 5 measurments , so H will be 5*(rows of state)
	///Note that the measurment is the sensor reading and its variable
	/// Eqn : Y=Z-H*X 
	this->H = Eigen::MatrixXd::Zero(number_of_measurment,d);     ///Measurment function 

	/// R will be same dimention as measurment 
	/// Eqn : S = H*P*H-transpose + R
	/// if we take 2 measurment so Z will be 2*1 and each measurment has its error
	/// this will lead R to be 2*1 also
	/// in our case we use one measurment 
	this->R = Eigen::MatrixXd::Zero(number_of_measurment,1);         ///Measurment uncertainty
	this->S = Eigen::MatrixXd::Zero(number_of_measurment,1);         ///Helper matrix
	this->I = Eigen::MatrixXd::Zero(d, d);


	this->Z = Eigen::MatrixXd::Zero(number_of_measurment, 1);  ///Measurment taken from sensor
	this->Y = Eigen::MatrixXd::Zero(number_of_measurment, 1);

	/// Eqn : P*H-transpose*S-inverse
	this->K = Eigen::MatrixXd::Zero(d, 1);   ///kalman gain
}

void::KalmanNDimentional::NewMeasurment(Eigen::MatrixXd Me)
{
	this->Z = Me;
}

void::KalmanNDimentional::NewMotion(Eigen::MatrixXd Mo)
{
	this->U = Mo;
}

void::KalmanNDimentional::MeasurmentUpdate()
{
	
	this->Y = (this->Z) - (this->H * this->X);
	this->S = (this->H * this->P * this->H.transpose()) + this->R;
	this->K = this->P * this->H.transpose() * this->S.inverse();

	//update the belief or the state
	this->X = this->X + (this->K * this->Y);

	//update the error in process
	this->P = (this->I - (this->K * this->H) ) * this->P;

}


void::KalmanNDimentional::MotionUpdate()
{
	//motion changes the state of the system and 
	//increases the uncertainty because we lose information

	this->X = (this->F * this->X) + this->U;


	this->P = this->F * this->P * this->F.transpose();
}

void::KalmanNDimentional::SetState(Eigen::MatrixXd x)
{
	this->X = x;
}

void::KalmanNDimentional::SetU(Eigen::MatrixXd u)
{
	this->U = u;
}

void::KalmanNDimentional::SetP(Eigen::MatrixXd p)
{
	this->P = p;
}
void::KalmanNDimentional::SetF(Eigen::MatrixXd f)
{
	this->F = f;
}

void::KalmanNDimentional::SetH(Eigen::MatrixXd h)
{
	this->H = h;
}

void::KalmanNDimentional::SetR(Eigen::MatrixXd r)
{
	this->R = r;
}

void::KalmanNDimentional::SetI(Eigen::MatrixXd i)
{
	this->I = i;
}


//Destructor
KalmanNDimentional::~KalmanNDimentional(){}