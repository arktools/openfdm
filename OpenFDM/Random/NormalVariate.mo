within OpenFDM.Random;

function normalvariate "normally distributed random variable"
    input Real mu "mean value";
    input Real sigma "standard deviation";
    input Seed si "input random seed";
    output Real x "gaussian random variate";
    output Seed so "output random seed";
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
