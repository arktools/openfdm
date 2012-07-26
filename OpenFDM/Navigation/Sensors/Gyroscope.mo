within OpenFDM.Navigation.Sensors;

model Gyroscope
    Real w_real(start=0);
    output Real w_meas(start=0);
    Real[3] seed(start={27,10089,61});
    parameter Real bias=0;
    parameter Real sigma=1;
    parameter Real samplePeriod=0.01;
protected
    discrete Real nextSampleTime(start=0);
    discrete Real noise(start=0);
algorithm
    when pre(nextSampleTime) <= time then
        nextSampleTime := pre(nextSampleTime) + samplePeriod;
        (noise,seed) := OpenFDM.Random.normalvariate(bias,sigma,seed);
        w_meas := w_real + noise;
        w_real := w_real;
    end when;
end Gyroscope;
