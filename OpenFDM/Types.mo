within OpenFDM;

type RealOutput = Modelica.Blocks.Interfaces.RealOutput;
type Angle_deg = Modelica.SIunits.Conversions.NonSIunits.Angle_deg;
type Length_ft = Conversions.NonSIunits.Length_ft;
type AngularVelocity_degs = Conversions.NonSIunits.AngularVelocity_degs;

record Geodetic
  import SI = Modelica.SIunits;
  SI.Angle latitude;  
  SI.Angle longitude;  
  SI.Length altitude;  
end Geodetic;

type ECEF = Modelica.SIunits.Position[3];

function ECEFToGeodetic
  input ECEF ecef;
  output Geodetic geod;
algorithm
  // TODO : implement
  geod.latitude := 0;
  geod.longitude := 0;
  geod.altitude := 0;
end ECEFToGeodetic;

function GeodeticToECEF
  input Geodetic geod;
  output ECEF ecef;
algorithm
  // TODO : implement
  ecef[1] := 0;
  ecef[2] := 0;
  ecef[3] := 0;
end GeodeticToECEF;


