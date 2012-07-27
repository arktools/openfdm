within Rand;

model RandTest
    //OpenFDM.Random.RandomNumber white;
    OpenFDM.Navigation.Sensors.Gyroscope scope(bias=0,sigma=10);
protected
    Real x(start = 100);
equation
    der(x) = -7.5*x;
    connect(scope.w_real,x);
end RandTest;

