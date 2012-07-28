within OpenFDM.Navigation.Sensors;

model IMUSensorBank
    input Real[3] w_real(each start=0);
    discrete Real[3] w_meas(each start=0);
    Seed seed1(start={1,2,3});
    Seed seed2(start={4,5,6});
    Seed seed3(start={7,8,9});
    parameter Real samplePeriod=0.005;
protected
    discrete Real nextSampleTime(start=0);
    discrete Real[3] noise(each start=0);
algorithm
    when pre(nextSampleTime) <= time then
        nextSampleTime := pre(nextSampleTime) + samplePeriod;
        (noise[1],seed1) := OpenFDM.Random.normalvariate(0,1,seed1);
        (noise[2],seed2) := OpenFDM.Random.normalvariate(0,1,seed2);
        (noise[3],seed3) := OpenFDM.Random.normalvariate(0,1,seed3);
        w_meas := w_real + noise;
    end when;
end IMUSensorBank;
