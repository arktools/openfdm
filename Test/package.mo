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

model World
  import Modelica.Mechanics.MultiBody;
  extends MultiBody.World;

end World;

model Environment
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  MultiBody.Interfaces.Frame frame;
  SI.Density rho "air density";  
  SI.Velocity airspeed "true airspeed";
protected
  SI.Velocity windECEF[3];
equation
  windECEF = {0,0,0}; // TODO
  rho = 1.225; // TODO
  airspeed = Vectors.norm(der(frame.r_0) - windECEF);
  // environment exerts no force torques
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Environment;

partial model Aerodynamics "abstract aerodynamics description, extends world force and torque from multibody"
  import SI = Modelica.SIunits;
  import Modelica.Mechanics.MultiBody;
  MultiBody.Interfaces.Frame_b frame_b;
protected
  Environment environment;
  MultiBody.Forces.WorldForceAndTorque forceTorque;
  SI.Force lift;
  SI.Force drag;
  SI.Force sideForce;
  SI.Torque rollMoment;
  SI.Torque pitchMoment;
  SI.Torque yawMoment;

  SI.Pressure qBar "average dynamics pressure";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "angle of attack derivative";
  SI.Angle beta "side slip angle";

equation
  connect(environment.frame,frame_b);
  connect(frame_b,forceTorque.frame_b);
  // TODO
  qBar = 0.5*environment.rho*environment.airspeed^2;
  alpha = 0.1;
  alphaDot = 0.0;
  beta = 0.1;

  // right hand set (forward, right, down)
  forceTorque.force = {-drag,sideForce,-lift};
  forceTorque.torque = {rollMoment,pitchMoment,yawMoment};
end Aerodynamics;

partial model AerodynamicsCoefficientBased "abstract coefficient based aerodynamics"
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
  lift = cL*qBar*s;
  drag = cD*qBar*s;
  sideForce = cC*qBar*s;
  rollMoment = cl*qBar*b*s;
  pitchMoment = cm*qBar*cBar*s;
  yawMoment = cn*qBar*b*s;
end AerodynamicsCoefficientBased;

block AerodynamicsCoefficientBasedBlock
  extends AerodynamicsCoefficientBased;
  import Modelica.Blocks.Interfaces.RealInput;
  input RealInput u[6];
equation
  u = {cL,cD,cC,cl,cm,cn};
end AerodynamicsCoefficientBasedBlock;

model TestAerodynamics
  import Modelica.Mechanics.MultiBody;
  inner World world;
  MultiBody.Parts.BodyShape body;
  AerodynamicsCoefficientBasedBlock aerodynamics(cBar=1,b=1,s=1);
  Modelica.Blocks.Sources.Constant coefs[6](k=0.0001*{1,1,1,1,1,1});
equation
  connect(body.frame_a,aerodynamics.frame_b);
  connect(coefs.y,aerodynamics.u);
end TestAerodynamics;

//model Aircraft
//
//  import Modelica.Mechanics.MultiBody.World;
//  import Modelica.Mechanics.MultiBody.Parts.BodyShape;
//  import Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque;
//
//  inner World world;
//  BodyShape body;
//  WorldForceAndTorque aerodynamics(force={0,0,0},torque={0,0,0});
//equation
//  connect(aerodynamics.frame_b,body.frame_a);
//end Test1;
//
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
