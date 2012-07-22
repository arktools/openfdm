package Test

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

model Atmosphere "atmosphere for multibody frame"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  MultiBody.Interfaces.Frame frame;
  SI.Density rho "air density";  
  SI.Velocity airspeed "true airspeed";
  SI.Angle alpha "angle of attack";
  SI.Angle beta "side slip angle";
  SI.Pressure qBar "average dynamics pressure";
protected
  SI.Velocity wind_ECEF[3];
  SI.Velocity vRelative_ECEF[3];
  SI.Velocity vRelative_XYZ[3];
equation
  rho = 1.225;
  wind_ECEF = {0,0,0};
  vRelative_ECEF = der(frame.r_0) - wind_ECEF;
  vRelative_XYZ = MultiBody.Frames.resolve2(frame.R,vRelative_ECEF);
  airspeed = Vectors.norm(vRelative_ECEF);

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if (airspeed < 0.01) then
    alpha = 0;
    beta = 0;
  else
    alpha = atan2(vRelative_XYZ[3],sqrt(vRelative_XYZ[1]^2+vRelative_XYZ[2]^2));
    beta = atan2(vRelative_XYZ[2],vRelative_XYZ[1]);
  end if;

  qBar = 0.5*rho*airspeed^2;

  // atmosphere exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Atmosphere;

partial model Aerodynamics "aerodynamic force/torque with multibody frame connector"
  import SI = Modelica.SIunits;
  import Modelica.Mechanics.MultiBody;
  MultiBody.Interfaces.Frame_b frame_b;
protected
  Atmosphere atmosphere;
  MultiBody.Forces.WorldForceAndTorque forceTorque;
  SI.Force lift;
  SI.Force drag;
  SI.Force sideForce;
  SI.Torque rollMoment;
  SI.Torque pitchMoment;
  SI.Torque yawMoment;
equation
  connect(atmosphere.frame,frame_b);
  connect(frame_b,forceTorque.frame_b);
  // right hand set (forward, right, down)
  // TODO check frames, gravity currently in 2nd comp
  // of world frame, this doesn't match NED
  forceTorque.force = {-drag,sideForce,-lift};
  forceTorque.torque = {rollMoment,pitchMoment,yawMoment};
end Aerodynamics;

partial model AerodynamicsCoefficientBased "coefficient based aerodynamics"
  import SI = Modelica.SIunits;
  extends Aerodynamics;
  parameter SI.Area s "reference area";
  parameter SI.Length b "span";
  parameter SI.Length cBar "mean chord";
protected
  Real cL "lift coefficient";
  Real cD "drag coefficient";
  Real cC "cross-wind coefficient";
  Real cl "roll moment coefficient";
  Real cm "pitch moment coefficient";
  Real cn "yaw moment coefficient";
equation
  lift = cL*atmosphere.qBar*s;
  drag = cD*atmosphere.qBar*s;
  sideForce = cC*atmosphere.qBar*s;
  rollMoment = cl*atmosphere.qBar*b*s;
  pitchMoment = cm*atmosphere.qBar*cBar*s;
  yawMoment = cn*atmosphere.qBar*b*s;
end AerodynamicsCoefficientBased;

block AerodynamicsCoefficientBasedBlock
  extends AerodynamicsCoefficientBased;
  import Modelica.Blocks.Interfaces.RealInput;
  input RealInput u[6];
equation
  u = {cL,cD,cC,cl,cm,cn};
end AerodynamicsCoefficientBasedBlock;

block AerodynamicCoefficientsTable
end AerodynamicCoefficientsTable;

model TestAerodynamics
  import Modelica.Mechanics.MultiBody;
  inner MultiBody.World world;
  MultiBody.Parts.BodyShape body(
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r={0.4,0,0},
    r_CM={0.2,0,0},
    width=0.05,
    r_0(start={0.2,-0.5,0.1}, fixed=true),
    v_0(fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    angles_start={0,0,0}*0.174532925199433);
  AerodynamicsCoefficientBasedBlock aerodynamics(
    cBar=1,
    b=1,
    s=1);
  Modelica.Blocks.Sources.Constant coefs[6](
    k=0.0*{1,1,1,1,1,1});
equation
  connect(body.frame_a,aerodynamics.frame_b);
  connect(coefs.y,aerodynamics.u);
end TestAerodynamics;

//model F16 extends Aircraft(
//  body(m=1
//    I_11=1,
//    I_22=1,
//    I_33=1,
//    r={0.4,0,0},
//    r_CM={0.2,0,0},
//    width=0.05,
//    r_0(start={0.2,-0.5,0.1}, fixed=true),
//    v_0(fixed=true),
//    angles_fixed=true,
//    w_0_fixed=true,
//    angles_start={0,0,0}*0.174532925199433)
//
//
end Test;

// vim:ts=2:sw=2:expandtab:
