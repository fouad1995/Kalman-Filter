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
mmmmmmmmmmm , so we created a gaussian shape !! <br>
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

#### Now we are ready to apply one dimentional Kalman Filter and use it in prediction 
