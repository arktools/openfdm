within OpenFDM.Navigation.Sensors;

model Sensor
    input Real real(start=0);
    discrete output Real meas(start=0);
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
        meas := real + noise;
    end when;
end Sensor;
