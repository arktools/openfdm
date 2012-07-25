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

function random "Pseudo random number generator" 
    input Real[3] seed;
    output Real x;
    output Real[3] outSeed;
algorithm 
    outSeed[1] := rem((171*seed[1]), 30269);
    outSeed[2] := rem((172*seed[2]), 30307);
    outSeed[3] := rem((170*seed[3]), 30323);
    for i in 1:3 loop 
        if outSeed[i] == 0 then 
            outSeed[i] := 1; 
        end if; 
    end for;

    x := rem((outSeed[1]/30269.0+outSeed[2]/30307.0 + outSeed[3]/30323.0), 1.0);
end random;
 
function normalvariate "normally distributed random variable"
    input Real mu "mean value";
    input Real sigma "standard deviation";
    input Real si[3] "input random seed";
    output Real x "gaussian random variate";
    output Real so[3] "output random seed";
protected
    constant Real NV_MAGICCONST=4*exp(-0.5)/sqrt(2.0);
    Real s1[3], s2[3];
    Real z, zz, u1, u2;
    Boolean breakL = false;
algorithm
    s1 := si;
    u2 := 1;
    while not breakL loop
        (u1,s2) := random(s1);
        (u2,s1) := random(s2);
        z := NV_MAGICCONST*(u1-0.5)/u2;
        zz := z*z/4.0;
        breakL := zz <= (- log(u2));
    end while;
    x := mu + z*sigma;
    so := s1;
 end normalvariate;

