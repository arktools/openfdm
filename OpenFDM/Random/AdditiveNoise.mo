within OpenFDM.Random;

block RandomNumber 
    extends Modelica.Blocks.Interfaces.DiscreteBlock;
    annotation(experiment(StartTime = 0.0, StopTime = 15, Tolerance = 1e-006));

    Real seed[3](start = {48, 51, 73});
    output Real x(start = 0);
algorithm
    when sampleTrigger then
        (x, seed) := normalvariate(0,1,seed);
//        seed := seed;
    end when;
end RandomNumber;
