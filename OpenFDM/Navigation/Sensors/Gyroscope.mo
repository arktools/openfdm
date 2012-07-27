within OpenFDM.Navigation.Sensors;

model Gyroscope
//    input SI.AngularVelocity w_real;
//    output discrete SI.AngularVelocity w_meas(start=0);
//    Real[3] seed(start={27,19,61});
//    parameter Real bias=0;
//    parameter Real sigma=1;
//    parameter Real samplePeriod=0.01;
protected
//    Real nextSampleTime(start=0);
//    Real noise;
equation
//    when pre(nextSampleTime) <= time then
//        nextSampleTime = pre(nextSampleTime) + samplePeriod;
//        (noise,seed) = normalvariate(bias,sigma,seed);
//        w_meas = w_real + noise.x;
//    end when;
end Gyroscope;
