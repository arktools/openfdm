within OpenFDM.Navigation.Sensors;

model IMUSensorBank
    input Real[2] real(each start=0);
    discrete output Real[2] meas(each start=0);
    Seed seed1(start={1,2,3});
    Seed seed2(start={4,5,6});
    parameter Real samplePeriod=0.01;
    parameter Real[2] sigma;
    parameter Real[2] mu;
protected
    discrete Real[2] noise(each start=0);
algorithm
    when sample(0,samplePeriod) then
        (noise[1],seed1) := OpenFDM.Random.normalvariate(0,1,seed1);
        (noise[2],seed2) := OpenFDM.Random.normalvariate(0,1,seed2);
        meas := real + noise;
    end when;
end IMUSensorBank;
