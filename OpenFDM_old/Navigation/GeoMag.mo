within OpenFDM.Navigation;

function geoMag
    input Real x1;
    input Real x2;
    output Real y1;
    output Real y2;
external "C" geoMag(x1,x2,y1,y2) annotation(Include="#include \"GeoMag/geoMag.c\"");
end geoMag;
