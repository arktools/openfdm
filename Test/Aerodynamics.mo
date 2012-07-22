within Test;

partial model Aerodynamics "aerodynamic force/torque with multibody frame connector"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  import Modelica.Blocks.Interfaces.RealInput;
  MultiBody.Interfaces.Frame_b frame_b;
  RealInput aileron;
  RealInput elevator;
  RealInput rudder;
  RealInput throttle;
  RealInput flap;
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

  // generic 1D table types 
  partial block Table1Ds
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
  end Table1DsElevator;

  block Table1DsFlap
    input Angle_deg flap;
  equation
    table.u = flap;
  end Table1DsFlap;

  // generic 2D table types
  partial block Table2Ds
    import Modelica.Blocks.Interfaces.RealOutput;
    RealOutput y;
    parameter Real table[:, :]=fill(0.0,0,2)
    CombiTable2D table(table=data);
  equation
    table.y = y[1];
  end Table2Ds;

  block Table2DAlphaFlap
    input Angle_deg alpha;   
    input Angle_deg flap;   
  equation
    table.u1 = alpha;
    table.u2 = flap;
  end Table2DAlphaFlap;

  block Table2DAlphaElevator
    input Angle_deg alpha;   
    input Angle_deg elevator;   
  equation
    table.u1 = alpha;
    table.u2 = elevator;
  end Table2DAlphaElevator;

  // lift coefficient tables
  CLge Table1DsHeight CLge;
  Table1DsAlpha CLwbh;
  Table1DsAlpha CLq;
  Table1DsAlpha CLad;
  Table1DsFlap CLdF1L;
  Table1DsFlap CLdF1R;
  Table1DsFlap CLdF2L;
  Table1DsFlap CLdF2R;
  Table1DsElevator CLDe;

  // drag coefficient tables
  Table1DsHeight CDge;
  Table1DsAlpha CD;
  Table2DAlphaFlap CdDf1L;
  Table2DAlphaFlap CdDf1R;
  Table2DAlphaFlap CdDf2L;
  Table2DAlphaFlap CdDf2R;
  Table2DAlphaElevator CdDe;

equation
  // lift
  connect(CLge.height,to_ft(env.agl));  
  connect(CDge.height,to_deg(env.agl));  
  connect(CLwbh.alpha,to_deg(alpha));  
  connect(CLq.alpha,to_deg(alpha));  
  connect(CLad.alpha,to_deg(alpha));  
  connect(CLad.alpha,to_deg(alpha));  
  connect(CLdF1L.flap,to_deg(flap));  
  connect(CLdF1R.flap,to_deg(flap));  
  connect(CLdF2L.flap,to_deg(flap));  
  connect(CLdF2R.flap,to_deg(flap));  
  connect(CLDe.elevator,to_deg(elevator));  
  cL = CLge.y*CLwbh.y +
       CLq.y*to_deg(q)*c/(2*vt) +
       CLad.y*to_deg(alphaDot)*c/(2*vt) +
       CLdF1L.y + CLdF1R.y +
       CLdF2L.y + CLdF2R.y +
       CLDe.y;
  // drag
  connect(CDge.height,to_ft(env.agl));
  connect(CD.alpha,to_deg(alpha));
  connect(CdDf1L.alpha,to_deg(alpha));
  connect(CdDf1L.flap,to_deg(flap));
  connect(CdDf1R.alpha,to_deg(alpha));
  connect(CdDf1R.flap,to_deg(flap));
  connect(CdDf2L.alpha,to_deg(alpha));
  connect(CdDf2L.flap,to_deg(flap));
  connect(CdDf2R.alpha,to_deg(alpha));
  connect(CdDf2R.flap,to_deg(flap));
  connect(CdDe.alpha,to_deg(alpha));
  connect(CdDe.elevator,to_deg(elevator));
  cD = CDge.y*CD.y + 
       CdDf1L.y + CdDf1R.y +
       CdDf2L.y + CdDf2R.y + CdDe.y;
  // side force
  connect(Cyb.alpha, to_deg(alpha));
  connect(Cyp.alpha, to_deg(alpha));
  
  cC = Cyb.y*to_deg(beta) +
       Cyp.y*to_deg(p)*b/(2*vt); 
  //3 more values not calculated by datcom: Cyr, CyDr, CyDa.
  
  // roll moment coefficient
  connect(Clb.alpha, to_deg(alpha));

  cl = Clb.y*b*to_deg(beta) +
       Clp.y*to_deg(p)*b*;
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

// vim:ts=2:sw=2:expandtab:
