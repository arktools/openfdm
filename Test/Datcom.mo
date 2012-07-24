within Test;

model AerodynamicBodyDatcom

  extends AerodynamicBodyCoefficientBased;
  import Modelica.Blocks.Tables.CombiTable1Ds;
  import Modelica.Blocks.Tables.CombiTable2D;
  import Modelica.SIunits.Conversions.*;
  import Test.Conversions.NonSIunits.*;

protected

  // generic 1D table types 
  partial block Table1Ds
    import Modelica.Blocks.Interfaces.RealOutput;
    RealOutput y;
    parameter Real data[:, :] = fill(0.0,0,2);
    CombiTable1Ds table(
        table=data,
        smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative);
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
    CombiTable2D table(
        table=data,
        smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative);
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

equation  

  // lift tables
  connect(CLge.height,agl_ft);  
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
  connect(CnDa.aileron, aileron_deg);

algorithm

  if (vt>vtTol) then 
    cL :=
//        CLge.y*CLwbh.y +
//        CLq.y*to_degs(aero_q)*cBar/(2*vt) +
//        CLad.y*to_degs(alphaDot)*cBar/(2*vt) +
//        CLdF1L.y + CLdF1R.y +
//        CLdF2L.y + CLdF2R.y +
//        CLDe.y +
        0;
    cD := 
//        CDge.y*CD.y +
//        CdDf1L.y + CdDf1R.y +
//        CdDf2L.y + CdDf2R.y + CdDe.y +
        0;
    cC :=
//        Cyb.y*to_deg(beta) +
//        Cyp.y*to_degs(aero_p)*b/(2*vt) +
        0;
       //3 more values not calculated by datcom: Cyr, CyDr, CyDa.
    cl := 
//       Clb.y*b*to_deg(beta) +
//       Clp.y*aero_p*b^2/(2*vt) + //p in rad?? TODO
//       Clr.y*aero_r*b^2/(2*vt) +
//       ClDs4.y*b +
//       CldF1 * (CLdF1R.y - CLdF1L.y) +
//       CldF2 * (CLdF2R.y - CLdF2L.y) +
//       ClDr*to_deg(rudder)*b + 
//          ClDr not calculated by datcom
       0;
    cm := 
//       Cm_basic.y*cBar +
//       Cmq.y*aero_q*cBar^2/(2*vt) +
//       Cmadot.y*cBar^2/(2*vt)*to_degs(alphaDot) +
//       CmDe.y*cBar +
//       CmDf1L.y* cBar +
//       CmDf1R.y* cBar +
//       CmDf2L.y* cBar +
//       CmDf2R.y* cBar +
       0;
    cn := 
//       Cnb.y*b*to_deg(beta) +
//       Cnp.y*b^2/(2*vt)*aero_p +
//       Cnr.y*b^2/(2*vt)*aero_r +
//       CnDf1*(CdDf1R.y - CdDf1L.y) +
//       CnDf2*(CdDf2R.y - CdDf2L.y) +
//       CnDa.y*b +
//       CnDr*b*rudder + //CnDr is not calculated by DATCOM
       0;
  else
    cL := 0;
    cD := 0;
    cC := 0;
    cl := 0;
    cm := 0;
    cn := 0;
  end if;
end AerodynamicBodyDatcom;

model AerodynamicBodyDatcom_Null
  constant Real test1D[:,:] = {{ 0,   0},
                               {90,   0}};
  constant Real test2D[:,:] =  {{ 0,  0, 90},
                                { 0,  0,  0},
                                {90,  0,  0}};
  extends AerodynamicBodyDatcom(  
    // lift coefficient tables
    CLge.data = test1D,
    CLwbh.data = test1D,
    CLq.data = test1D,
    CLad.data = test1D,
    CLdF1L.data = test1D,
    CLdF1R.data = test1D,
    CLdF2L.data = test1D,
    CLdF2R.data = test1D,
    CLDe.data = test1D,

    // drag coefficient tables
    CDge.data = test1D,
    CD.data = test1D,
    CdDf1L.data = test2D,
    CdDf1R.data = test2D,
    CdDf2L.data = test2D,
    CdDf2R.data = test2D,
    CdDe.data = test2D,
     
    // side force coefficient tables
    Cyb.data = test1D,
    Cyp.data = test1D,

    // roll moment coefficient tables
    Clb.data = test1D,
    Clp.data = test1D,
    Clr.data = test1D,
    ClDs4.data = test1D,
    CldF1 = 0,
    CldF2 = 0,
    ClDr = 0,
    
    // pitch moment coefficient tables
    Cm_basic.data = test1D,
    Cmq.data = test1D,
    Cmadot.data = test1D,
    CmDe.data = test1D,
    CmDf1L.data = test1D,
    CmDf1R.data = test1D,
    CmDf2L.data = test1D,
    CmDf2R.data = test1D,

    // yaw moment coefficient tables
    Cnb.data = test1D,
    Cnp.data = test1D,
    Cnr.data = test1D,
    CnDf1 = 0,
    CnDf2 = 0,
    CnDa.data = test2D,
    CnDr = 0
    );
end AerodynamicBodyDatcom_Null;

model TestAerodynamicBodyDatcom
  inner Modelica.Mechanics.MultiBody.World world(n={0,0,1});
  AerodynamicBodyDatcom_B_737 body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    aero_rp = {1,0,0},
    s = 1,
    b = 1,
    cBar = 1,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r={0.4,0,0},
    r_CM={0.2,0,0},
    width=0.05,
    r_0(start={0.2,-0.5,0.1}, fixed=true),
    v_0(start={1,0,0}, fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    angles_start={0,0,0}*0.174532925199433,
    sequence_angleStates = {3,2,1},
    sequence_start = {3,2,1},
    useQuaternions = false);
end TestAerodynamicBodyDatcom;
