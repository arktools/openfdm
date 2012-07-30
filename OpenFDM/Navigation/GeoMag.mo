within OpenFDM.Navigation;

function geoMag
    input Real x1;
    input Real x2;
    output Real y;
external "C" y = atan2(x1,x2);
end geoMag;
