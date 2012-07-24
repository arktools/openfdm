within Test;

partial model AerodynamicBody "aerodynamic force/torque with multibody frame connector"
  extends Modelica.Mechanics.MultiBody.Parts.BodyShape(
    r = aero_rp,
    angles_fixed=true,
    w_0_fixed=true,
    sequence_angleStates = {3,2,1},
    sequence_start = {3,2,1},
    useQuaternions = false);
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.SIunits.Conversions.*;
  import Test.Conversions.NonSIunits.*;
  RealInput aileron;
  RealInput elevator;
  RealInput rudder;
  RealInput flap;
  parameter Real[3] aero_rp "aerodynamic reference point";
  parameter Real vtTol=0.001 "Velocity above which aerodynamics are enabled";
protected
  Environment env;
  MultiBody.Forces.WorldForceAndTorque forceTorque(
    resolveInFrame=MultiBody.Types.ResolveInFrameB.frame_b);
  SI.Force lift;
  SI.Force drag;
  SI.Force sideForce;
  SI.Torque rollMoment;
  SI.Torque pitchMoment;
  SI.Torque yawMoment;

  SI.Velocity vt "true airspeed";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "angle of attack derivative";
  SI.Angle beta "side slip angle";
  SI.Pressure qBar "average dynamics pressure";
  SI.AngularVelocity aero_p "roll rate";
  SI.AngularVelocity aero_q "pitch rate";
  SI.AngularVelocity aero_r "yaw rate";

  SI.Velocity vRelative_ECEF[3];
  SI.Velocity vRelative_XYZ[3];
  SI.Velocity aRelative_ECEF[3];
  SI.Velocity aRelative_XYZ[3];

  Angle_deg roll_deg;
  Angle_deg pitch_deg;
  Angle_deg yaw_deg;

  Angle_deg alpha_deg;
  Angle_deg beta_deg;
  Angle_deg beta_deg_abs;
  Angle_deg aileron_deg;
  Angle_deg elevator_deg;
  Angle_deg flap_deg;
  Angle_deg rudder_deg;
  Length_ft agl_ft;
  
equation
  // conversion
  yaw_deg = to_deg(body.phi[1]);
  pitch_deg = to_deg(body.phi[2]);
  roll_deg = to_deg(body.phi[3]);
  alpha_deg = to_deg(alpha); 
  beta_deg = to_deg(beta);
  beta_deg_abs = abs(beta_deg);
  flap_deg = to_deg(flap); 
  elevator_deg = to_deg(elevator); 
  aileron_deg = to_deg(aileron);
  rudder_deg = to_deg(rudder); 
  agl_ft = Conversions.NonSIunits.to_ft(env.agl);

  // TODO
  vRelative_ECEF = v_0 - env.wind_ECEF;
  aRelative_ECEF = a_0; // TODO: - der(env.wind_ECEF);
  vRelative_XYZ = resolve2(frame_a.R,vRelative_ECEF);
  aRelative_XYZ = resolve2(frame_a.R,aRelative_ECEF);
  vt = Vectors.norm(vRelative_XYZ);
  {aero_p,aero_q,aero_r} = angularVelocity2(frame_b.R);

  alpha = atan2(vRelative_XYZ[3],vRelative_XYZ[1]);
  qBar = 0.5*env.rho*vt^2;

  // avoid singularity in side slip angle calc
  if (vt > vtTol) then
    beta = asin(vRelative_XYZ[2]/vt);
  else
    beta = 0;
  end if;

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if ( (vRelative_XYZ[1]^2 + vRelative_XYZ[3]^2) > vtTol) then
    alphaDot = (vRelative_XYZ[1]*aRelative_XYZ[3]-vRelative_XYZ[3]*aRelative_XYZ[1])/(vRelative_XYZ[1]^2 + vRelative_XYZ[3]^2); //stevens & lewis pg 78
  else
    alphaDot = 0;
  end if;

  connect(env.frame,frame_b);
  connect(frame_b,forceTorque.frame_b);
  // right hand set (forward, right, down)
  forceTorque.force = {-drag,sideForce,-lift};
  forceTorque.torque = {rollMoment,pitchMoment,yawMoment};
end AerodynamicBody;

model AerodynamicBodyCoefficientBased "coefficient based aerodynamics"
  import SI = Modelica.SIunits;
  extends AerodynamicBody;
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
end AerodynamicBodyCoefficientBased;

model AerodynamicBodySimple
  extends AerodynamicBodyCoefficientBased;

  // stall
  parameter Real alphaStall_deg = 20;
  parameter Real betaStall_deg = 20;

  // lift
  parameter Real cL0 = 0.1 "cl vs. aoa y-intercept";
  parameter Real cLa = 1.5/20.0 "cl vs. aoa slope";

  // drag
  parameter Real cD0 = 0.01 "drag polar y-intercept";
  parameter Real cDcL2 = 0.01 "cL^2 term in drag polar";

  // side force
  parameter Real cCb = 0.1/20.0 "side slip effect on side force";

  // roll moment
  parameter Real clp = 0.1 "roll damping";
  parameter Real cldA = .01/20.0 "aileron effect on roll";

  // pitch moment
  parameter Real cmq = 0.1 "pitch damping";
  parameter Real cma = 0.1 "aoa effect";
  parameter Real cmdE = 0.1/20.0 "elevator effect on pitch";

  // yaw moment
  parameter Real cnb = 0.1/20.0 "weather cocking stability";
  parameter Real cnr = 0.1 "yaw damping";
  parameter Real cndr = 0.1/20.0 "rudder effect on yaw";

protected

  function stallModel
    input Real angle;
    input Real stallAngle;
    output Real effective;
  algorithm
    if (angle < stallAngle) then
      effective := angle;
    else // stall
      effective := 0;
    end if; 
  end stallModel;

  Real alpha_deg_effective;
  Real beta_deg_effective;

equation

  // modify effective alpha/ beta for stall model
  alpha_deg_effective = stallModel(
    alpha_deg,alphaStall_deg);
  beta_deg_effective = stallModel(
    beta_deg,betaStall_deg);

  cL = // lift  
    cLa*alpha_deg_effective + cL0 + // aoa effect
    0; // allows other lines to be commented out

  cD = // drag 
    cDcL2*cL^2 + cD0 + // drag polar
    0; // allows other lines to be commented out

  cC = // side force
    cCb*beta_deg_effective + // beta effect
    0; // allows other lines to be commented out

  cl = // roll moment 
    (-clp)*aero_p + // damping
    cldA*aileron_deg + // control input
    0; // allows other lines to be commented out

  cm = // pitch moment
    (-cma)*alpha_deg_effective + // aoa effect
    (-cmq)*aero_q + // damping
    cmdE*elevator_deg + // control input
    0; // allows other lines to be commented out

  cn = // yaw moment
    (-cnb)*beta_deg_effective + // weather cocking
    (-cnr)*aero_r + // damping 
    cndr*rudder_deg + // control input
    0; // allows other lines to be commented out

end AerodynamicBodySimple;

model AerodynamicBodyCoefficientBasedBlock
  extends AerodynamicBodyCoefficientBased;
  import Modelica.Blocks.Interfaces.RealInput;
  RealInput u[6];
equation
  u = {cL,cD,cC,cl,cm,cn};
end AerodynamicBodyCoefficientBasedBlock;

model TestAerodynamicBodyBlock
  inner Modelica.Mechanics.MultiBody.World world(
    n={0,0,1});

  Modelica.Blocks.Sources.Constant coefs[6](k={1,1,1,1,1,1}*0.000000001);
  AerodynamicBodyCoefficientBasedBlock body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    r = {0,0,0}, // aerodynamic reference point
    s = 1,
    b = 1,
    cBar = 1,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r_CM={0,0,0},
    width=0.05,
    r_0(start={0,0,0}, fixed=true),
    v_0(start={10,0,0}, fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    sequence_angleStates = {3,2,1},
    sequence_start = {3,2,1},
    angles_start={0,0,0}*0.174532925199433,
    useQuaternions=false);
equation
  connect(coefs.y,body.u);
end TestAerodynamicBodyBlock;

model TestAerodynamicBodySimple
  inner Modelica.Mechanics.MultiBody.World world(
    n={0,0,1});
  AerodynamicBodySimple body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    r = {0,0,0}, // aerodynamic reference point
    s = 1,
    b = 1,
    cBar = 1,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r_CM={0,0,0},
    width=0.05,
    r_0(start={0,0,0}, fixed=true),
    v_0(start={10,0,0}, fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    sequence_angleStates = {3,2,1},
    sequence_start = {3,2,1},
    angles_start={0,0,0}*0.174532925199433,
    useQuaternions=false);
end TestAerodynamicBodySimple;

// vim:ts=2:sw=2:expandtab:
