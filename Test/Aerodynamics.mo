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
  RealInput flap;
  parameter Real vtTol=0.1 "Velocity above which aerodynamics are enabled";
protected
  Environment env;
  MultiBody.Forces.WorldForceAndTorque forceTorque;
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
  SI.AngularVelocity p "roll rate";
  SI.AngularVelocity q "pitch rate";
  SI.AngularVelocity r "yaw rate";

  SI.Velocity vRelative_ECEF[3];
  SI.Velocity vRelative_XYZ[3];

equation

  // TODO
  vRelative_ECEF = der(frame_b.r_0) - env.wind_ECEF;
  vRelative_XYZ = resolve2(frame_b.R,vRelative_ECEF);
  vt = Vectors.norm(vRelative_ECEF);
  {p,q,r} = angularVelocity2(frame_b.R);

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if (vt < 0.01) then
    alphaDot = 0;
    alpha = 0;
    beta = 0;
    qBar = 0;
  else
    alpha = atan2(vRelative_XYZ[3],sqrt(vRelative_XYZ[1]^2+vRelative_XYZ[2]^2));
    alphaDot = der(alpha);
    beta = atan2(vRelative_XYZ[2],vRelative_XYZ[1]);
    qBar = 0.5*env.rho*vt^2;
  end if;

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
  import Modelica.Blocks.Tables.CombiTable2D;
  import Conversions.NonSIunits.*;
  import Conversions.NonSIunits.to_ft;
  import Modelica.SIunits.Conversions.*;
  import Test.Conversions.NonSIunits.*;

protected

  // generic 1D table types 
  partial block Table1Ds
    import Modelica.Blocks.Interfaces.RealOutput;
    RealOutput y;
    parameter Real data[:, :] = fill(0.0,0,2);
    CombiTable1Ds table(table=data);
  equation
    table.y[1] = y;
  end Table1Ds;

  block Table1DsHeight
    extends Table1Ds;
    input Length_ft height;
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
    extends Table1Ds;
    input AngularVelocity_degs alphaDot;
  equation
    table.u = alphaDot;
  end Table1DsAlphaDot;

  block Table1DsElevator
    extends Table1Ds;
    input Angle_deg elevator;
  equation
    table.u = elevator;
  end Table1DsElevator;

  block Table1DsFlap
    extends Table1Ds;
    input Angle_deg flap;
  equation
    table.u = flap;
  end Table1DsFlap;
  
  block Table1DsAileron
    extends Table1Ds;
    input Angle_deg aileron;
  equation
    table.u = aileron;
  end Table1DsAileron;

  // generic 2D table types
  partial block Table2Ds
    import Modelica.Blocks.Interfaces.RealOutput;
    parameter Real data[:, :]  = fill(0.0,0,2);
    RealOutput y;
    CombiTable2D table(table=data);
  equation
    table.y = y;
  end Table2Ds;

  block Table2DAlphaFlap
    extends Table2Ds;
    input Angle_deg alpha;   
    input Angle_deg flap;   
  equation
    table.u1 = alpha;
    table.u2 = flap;
  end Table2DAlphaFlap;

  block Table2DAlphaElevator
    extends Table2Ds;
    input Angle_deg alpha;   
    input Angle_deg elevator;   
  equation
    table.u1 = alpha;
    table.u2 = elevator;
  end Table2DAlphaElevator;

  block Table2DAlphaAileron
    extends Table2Ds;
    input Angle_deg alpha;   
    input Angle_deg aileron;   
  equation
    table.u1 = alpha;
    table.u2 = aileron;
  end Table2DAlphaAileron;

  // lift coefficient tables
  Table1DsHeight CLge;
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
   
  // side force coefficient tables
  Table1DsAlpha Cyb;
  Table1DsAlpha Cyp;

  // roll moment coefficient tables
  Table1DsAlpha Clb;
  Table1DsAlpha Clp;
  Table1DsAlpha Clr;
  Table1DsAileron ClDs4;
  parameter Real CldF1;
  parameter Real CldF2;
  parameter Real ClDr; 
  
  // pitch moment coefficient tables
  Table1DsAlpha Cm_basic;
  Table1DsAlpha Cmq;
  Table1DsAlpha Cmadot;
  Table1DsFlap  CmDe;
  Table1DsFlap  CmDf1L;
  Table1DsFlap  CmDf1R;
  Table1DsFlap  CmDf2L;
  Table1DsFlap  CmDf2R;

  // yaw moment coefficient tables
  Table1DsAlpha Cnb;
  Table1DsAlpha Cnp;
  Table1DsAlpha Cnr;
  parameter Real CnDf1;
  parameter Real CnDf2;
  Table2DAlphaAileron CnDa;
  parameter Real CnDr;
  
  Angle_deg alpha_deg;
  Angle_deg aileron_deg;
  Angle_deg elevator_deg;
  Angle_deg flap_deg;
  Length_ft agl_ft;
  
equation

  // conversion
  alpha_deg = to_deg(alpha); 
  flap_deg = to_deg(flap); 
  elevator_deg = to_deg(elevator); 
  aileron_deg = to_deg(aileron); 
  agl_ft = Conversions.NonSIunits.to_ft(env.agl);

  // lift tables
  connect(CLge.height,agl_ft);  
  connect(CDge.height,agl_ft);  
  connect(CLwbh.alpha,alpha_deg);  
  connect(CLq.alpha,alpha_deg);  
  connect(CLad.alpha,alpha_deg);  
  connect(CLad.alpha,alpha_deg);  
  connect(CLdF1L.flap,flap_deg);  
  connect(CLdF1R.flap,flap_deg);  
  connect(CLdF2L.flap,flap_deg);  
  connect(CLdF2R.flap,flap_deg);  
  connect(CLDe.elevator,elevator_deg);  

  // drag tables
  connect(CDge.height,agl_ft);
  connect(CD.alpha,alpha_deg);
  connect(CdDf1L.alpha,alpha_deg);
  connect(CdDf1L.flap,flap_deg);
  connect(CdDf1R.alpha,alpha_deg);
  connect(CdDf1R.flap,flap_deg);
  connect(CdDf2L.alpha,alpha_deg);
  connect(CdDf2L.flap,flap_deg);
  connect(CdDf2R.alpha,alpha_deg);
  connect(CdDf2R.flap,flap_deg);
  connect(CdDe.alpha,alpha_deg);
  connect(CdDe.elevator,elevator_deg);

  // side force tables
  connect(Cyb.alpha, alpha_deg);
  connect(Cyp.alpha, alpha_deg);

  // roll moment coefficient
  connect(Clb.alpha, alpha_deg);
  connect(Clp.alpha, alpha_deg);
  connect(Clr.alpha, alpha_deg);
  connect(ClDs4.aileron, aileron_deg);

  // pitch moment coefficient
  connect(Cm_basic.alpha, alpha_deg);
  connect(Cmq.alpha, alpha_deg);
  connect(Cmadot.alpha, alpha_deg);
  connect(CmDe.flap, flap_deg);
  connect(CmDf1L.flap, flap_deg);
  connect(CmDf1R.flap, flap_deg);
  connect(CmDf2L.flap, flap_deg);
  connect(CmDf2R.flap, flap_deg);

  // yaw moment coefficient
  connect(Cnb.alpha, alpha_deg);
  connect(Cnp.alpha, alpha_deg);
  connect(Cnr.alpha, alpha_deg);
  connect(CnDa.alpha, alpha_deg);
  connect(CnDa.alpha, aileron_deg);

algorithm
  if (vt>vtTol) then 
    cL := CLge.y*CLwbh.y +
       CLq.y*to_degs(q)*cBar/(2*vt) +
       CLad.y*to_degs(der(alpha))*cBar/(2*vt) +
       CLdF1L.y + CLdF1R.y +
       CLdF2L.y + CLdF2R.y +
       CLDe.y;
    cD := CDge.y*CD.y + 
       CdDf1L.y + CdDf1R.y +
       CdDf2L.y + CdDf2R.y + CdDe.y;
    cC := Cyb.y*to_deg(beta) +
       Cyp.y*to_degs(p)*b/(2*vt); 
       //3 more values not calculated by datcom: Cyr, CyDr, CyDa.
    cl := Clb.y*b*to_deg(beta) +
       Clp.y*p*b^2/(2*vt) + //p in rad?? TODO
       Clr.y*r*b^2/(2*vt) +
       ClDs4.y*b +
       CldF1 * (CLdF1R.y - CLdF1L.y) +
       CldF2 * (CLdF2R.y - CLdF2L.y) +
       ClDr*to_deg(rudder)*b;//ClDr not calculated by datcom
    cm := Cm_basic.y*cBar +
       Cmq.y*q*cBar^2/(2*vt) +
       Cmadot.y*cBar^2/(2*vt)*to_degs(der(alpha)) +
       CmDe.y*cBar +
       CmDf1L.y* cBar +
       CmDf1R.y* cBar +
       CmDf2L.y* cBar +
       CmDf2R.y* cBar;
    cn := Cnb.y*b*to_deg(beta) +
       Cnp.y*b^2/(2*vt)*p +
       Cnr.y*b^2/(2*vt)*r +
       CnDf1*(CdDf1R.y - CdDf1L.y) + 
       CnDf2*(CdDf2R.y - CdDf2L.y) +
       CnDa.y*b +
       CnDr*b*rudder; //CnDr is not calculated by DATCOM
  else
    cL := 0;
    cD := 0;
    cC := 0;
    cl := 0;
    cm := 0;
    cn := 0;
  end if;
end AerodynamicsDatcom;

model AerodynamicsDatcom_Null
  inner Modelica.Mechanics.MultiBody.World world;
  constant Real empty[:,:] = {{0,0},{1,0}};
  extends AerodynamicsDatcom(  
    // controls
    flap=0,
    aileron=0,
    elevator=0,
    rudder=0,

    cBar=1,
    b=1,
    s=1,

    // lift coefficient tables
    CLge.data = empty,
    CLwbh.data = empty,
    CLq.data = empty,
    CLad.data = empty,
    CLdF1L.data = empty,
    CLdF1R.data = empty,
    CLdF2L.data = empty,
    CLdF2R.data = empty,
    CLDe.data = empty,

    // drag coefficient tables
    CDge.data = empty,
    CD.data = empty,
    CdDf1L.data = empty,
    CdDf1R.data = empty,
    CdDf2L.data = empty,
    CdDf2R.data = empty,
    CdDe.data = empty,
     
    // side force coefficient tables
    Cyb.data = empty,
    Cyp.data = empty,

    // roll moment coefficient tables
    Clb.data = empty,
    Clp.data = empty,
    Clr.data = empty,
    ClDs4.data = empty,
    CldF1 = 0,
    CldF2 = 0,
    ClDr = 0, 
    
    // pitch moment coefficient tables
    Cm_basic.data = empty,
    Cmq.data = empty,
    Cmadot.data = empty,
    CmDe.data = empty,
    CmDf1L.data = empty,
    CmDf1R.data = empty,
    CmDf2L.data = empty,
    CmDf2R.data = empty,

    // yaw moment coefficient tables
    Cnb.data = empty,
    Cnp.data = empty,
    Cnr.data = empty,
    CnDf1 = 0,
    CnDf2 = 0,
    CnDa.data = empty,
    CnDr = -0.1047e-2
    );
equation

end AerodynamicsDatcom_Null;

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
    v_0(start={1,1,1}, fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    angles_start={0,0,0}*0.174532925199433);
  AerodynamicsDatcom_Null aerodynamics;
equation
  connect(body.frame_a,aerodynamics.frame_b);
end TestAerodynamics;

model TestBody
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
end TestBody;

model TestAero
  extends Aerodynamics;
end TestAero;

// vim:ts=2:sw=2:expandtab:
