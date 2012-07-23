within OpenFDM.Navigation.Sensors;

model SensorNoise
	import Modelica.Math.Random.normalvariate;
	
	input Real value;
	input Real sig;
	input Real seed;
	
	output sensorValue;
	
	normalvariate Noise(mu=0, sigma=sig, si=seed);
	
equation
	sensorValue = value + Noise.x;
	
end SensorNoise;
