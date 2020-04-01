# Kalman-Filter
implementation of one and multidimensional Kalman filter using C++ with Eigen 


### Lets see how to use *KALMAN*

### we have 3 classes
- Gaussian ( represents 1d gaussian only )
- Kalman ( used only for one dimentional )
- KalmanNDimentional

first lets look at Gaussian class 

The main function of this class is to create a Gaussian shape with a specific ***mean*** and ***variance*** .
For example , when you create an object from Gaussian class `Gaussian x(10.0,5.0);` it will create a gaussian with mean 10.0 and variance = 5.0  like that : <br>
<p align="center">
<img src="https://github.com/fouad1995/Kalman-Filter/blob/master/Imgs/gaussian.png" width="300" height="300">
</p>
mmmmmmmmmmm , so we created a gaussian shape !!<br>
<p align="center">
<img src="https://github.com/fouad1995/Kalman-Filter/blob/master/Imgs/now what.gif" width="300" height="300">
</p>

In fact creating gaussian shapes only will not help , we need to do some operations like multiplying or adding two gaussian together
so there are two helper functions that do these operations.
<p align="center">
  
- `Gaussian MultiplyGaussian( Gaussian  & x,  Gaussian  & y);`<br>
</p>
This function takes two gaussian , multiply them together and return a new gaussian , this gaussian is more accurate than two gaussians (because its a compination of two sources) <br>
<p align="center">
<img src="https://github.com/fouad1995/Kalman-Filter/blob/master/Imgs/multiply gaussian.png" width="300" height="300"><br>
</p>
The Red one is the result and note that its more certain(low variance) than others and has a mean in between of two means <br><br>
<p align="center">

- `Gaussian AddGaussian( Gaussian  & x,  Gaussian  & y);`
</p>
Adding two gaussian means that we lose information so the new gaussian expected to has a large variance and large mean

<p align="center">
#### Now we are ready to apply one dimentional Kalman Filter and use it in prediction 
</p>
 ***Kalman Filter***  class has three important Gaussians as a private data members<br>

- Belief<br> This gaussian represents the state of the system , assume that our system is robot that is moving in 1-D 
so ***Belief gaussian*** will represent our confidence about the position of the robot<br>In other words , it tells us how the robot trusts its position 

- Measurment<br> when the robot takes a measurment ( sensor reading ) that tells some thing about his position (Not necessary the position itself) , this measurment is not accurate , it depends on the sensor that takes the reading so ***Measurment Gaussian*** represents our confidence about the measurment<br>In other words , it tells us how the robot trusts its sensors 

- Motion<br> This Gaussian represents the motion of the robot , the motion itself has error becasue that the things that responsible for robot motion like (motors ,PID ....etc) has its own error and this will casuse an error in motion for the robot<br>
For example , if the robot stays at position 2 and recieve a command like that (move 1 meter forward) , ideally if there is no error in motion the robot will go and stay at position 3 , but in reality the motion has error so the robot most likely to be at position 3 but also its likely to be around this position (for example it might be at any position between 2 and 4 like in figure below)<br>

<p align="center">
<img src="https://github.com/fouad1995/Kalman-Filter/blob/master/Imgs/gaussianMotion.png" width="500" height="500"><br>
</p>
