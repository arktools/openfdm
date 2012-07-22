package Test


package Conversions

  package NonSIunits

    type Length_ft = Real(
      final quantity="Length",
      final unit="ft") "Length in feet";

    function to_feet
      input Length meters;
      output Length_ft feet;
    algorithm
      feet = meters * 3.281;
    end to_feet;

    function from_feet
      input Length_ft feet;
      output Length meters;
    algorithm
      meters = 0.3048*feet;
    end from_feet;

    type AngularVelocity_degs = Real (
      final quantity="AngularVelocity",
      final unit="deg/s")
      "Angular velocity in degrees per second";

    function to_degs
      input AngularVelocity rads;
      output AngularVelocity_degs degs;
    algorithm
      degs := 57.2957795*rads;
    end to_degs;

    function from_degs
      input AngularVelocity_degs degs;
      output AngularVelocity rads;
    algorithm
      rads := 0.0174532925*degs;
    end from_degs;

  end NonSIunits;

end Conversions;

// declare some needed types
type RealOutput = Modelica.Blocks.Interfaces.RealOutput;
type Angle_deg = Modelica.SIunits.Conversions.NonSIunits.Angle_deg;
type Distance_ft = Conversions.NonSIunits.Distance_ft;
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

model Environment "environment for multibody frame"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  MultiBody.Interfaces.Frame frame;
  SI.Density rho "air density";  
  SI.Distance asl "altitude above sea level";
  SI.Distance agl "altitude above ground level";
  SI.Distance groundAsl "altitude of ground above sea level";
  SI.Velocity wind_ECEF[3] "wind vector";
equation
  asl = frame.r_0[3]; // TODO: should subtract radius of earth
  agl = asl - groundAsl; 

  // TODO
  rho = 1.225;
  wind_ECEF = {0,0,0};
  groundAltitude = 0;

  // envronment exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Environment;

partial model Aerodynamics "aerodynamic force/torque with multibody frame connector"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  MultiBody.Interfaces.Frame_b frame_b;
protected
  Environment env;
  MultiBody.Forces.WorldForceAndTorque forceTorque;
  SI.Force lift;
  SI.Force drag;
  SI.Force sideForce;
  SI.Torque rollMoment;
  SI.Torque pitchMoment;
  SI.Torque yawMoment;

  SI.Density rho "air density";  
  SI.Velocity vt "true airspeed";
  SI.Angle alpha "angle of attack";
  SI.Angle beta "side slip angle";
  SI.Pressure qBar "average dynamics pressure";
  SI.AngularVelocity p "roll rate";
  SI.AngularVelocity q "pitch rate";
  SI.AngularVelocity r "yaw rate";

protected
  SI.Velocity wind_ECEF[3];
  SI.Velocity vRelative_ECEF[3];
  SI.Velocity vRelative_XYZ[3];

equation

  // TODO
  vRelative_ECEF = der(frame_b.r_0) - env.wind_ECEF;
  vRelative_XYZ = resolve2(frame.R,vRelative_ECEF);
  vt = Vectors.norm(vRelative_ECEF);
  {p,q,r} = angularVelocity2(frame.R)

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if (vt < 0.01) then
    alpha = 0;
    beta = 0;
  else
    alpha = atan2(vRelative_XYZ[3],sqrt(vRelative_XYZ[1]^2+vRelative_XYZ[2]^2));
    beta = atan2(vRelative_XYZ[2],vRelative_XYZ[1]);
  end if;

  qBar = 0.5*env.rho*vt^2;

  // envronment exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Environment;


equation
  connect(env.frame,frame_b);
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
  RealInput u[6];
equation
  u = {cL,cD,cC,cl,cm,cn};
end AerodynamicsCoefficientBasedBlock;

model AerodynamicsDatcom

  extends AerodynamicsCoefficientBased;
  import Modelica.Blocks.Tables.CombiTable1Ds;
  import Conversions.NonSIunits.*;

  // generic table types 
  block Table1Ds
    import Modelica.Blocks.Interfaces.RealOutput;
    RealOutput y;
    parameter Real table[:, 2]=fill(0.0,0,2)
    CombiTable1Ds table(table=data);
  equation
    table.y = y[1];
  end Table1Ds;

  block Table1DsHeight
    extends Table1Ds;
    input Distance_ft height;
  equation
    table.u = height;
  end Table1DsHeight;

  block Table1DsAlpha
    extends Table1Ds;
    input Angle_deg alpha;
  equation
    table.u = alpha;
  end Table1DsAlpha;

  block Table1DsAlphaDot
    input AngularVelocity_degs alphaDot;
  equation
    table.u = alphaDot;
  end Table1DsAlphaDot;

  block Table1DsElevator
    input Angle_deg elevator;
  equation
    table.u = elevator;
  end Table1DsElevator

  block Table1DsFlaps
    input Angle_deg flaps;
  equation
    table.u = flaps;
  end Table1DsFlaps

  // actual table definitions
  CLge Table1DsHeight CLge;
  Table1DsHeight CDge;
  Table1DsAlpha CLwbh;
  Table1DsAlpha CLq;
  Table1DsAlpha CLad;
  Table1DsAlpha CLdF1L;

equation
  connect(CLge.height,to_ft(env.agl));  
  connect(CDge.height,to_deg(env.agl));  
  connect(CLwbh.alpha,to_deg(alpha));  
  connect(CLq.alpha,to_deg(alpha));  
  connect(CLad.alpha,to_deg(alpha));  
  connect(CLad.alpha,to_deg(alpha));  
  cL = cLge.y*cLalpha.y +
       cLq.y*to_degs(q)*c/(2*vt) +
       cLad*to_degs(alphaDot)*c/(2*vt) +
       cLdF1L = ;
  cD = cDge*;
  cC = 0;
  cl = 0;
  cm = 0;
  cn = 0;
end AerodynamicsDatcom;

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
  AerodynamicsDatcom aerodynamics(
    cL0Table(table={{0,0},{1,0}}),
    cBar=1,
    b=1,
    s=1);
equation
  connect(body.frame_a,aerodynamics.frame_b);
end TestAerodynamics;

end Test;

// vim:ts=2:sw=2:expandtab:
