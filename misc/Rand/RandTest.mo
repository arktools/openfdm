within Rand;

model RandTest
    //OpenFDM.Random.RandomNumber white;
    OpenFDM.Navigation.Sensors.IMUSensorBank gyros;
protected
    Real x[3](start = {100,75,50});
equation
    der(x) = -7.5*x;
    connect(gyros.w_real,x);
end RandTest;

