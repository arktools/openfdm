within OpenFDM.Test;

model RandTest
    OpenFDM.Navigation.KalmanFilter kf;
    OpenFDM.Navigation.Sensors.Sensor s(sigma=5);
protected
    Real x[4,1](start = {{100},{75},{50},{80}});
    parameter Real[4,4] A = {{0,1,0,0},{0,0,1,0},{0,0,0,1},{-24,-50,-35,-10}};
equation
    //x = x;
    der(x) = A*x;
    connect(x[1,1],s.real);
    connect(s.meas,kf.y[1,1]);
end RandTest;
