package JetPack
  model Atmosphere_SI
    constant Real rho = 1.225 "density [kg/m^3]";
    constant Real P = 101300 "Pressure [Pa]";
    constant Real T = 300 "temperature [K]";
    constant Real a = 300 "speed of sound [m/s]";
  end Atmosphere_SI;
  model ControlInput
    //Real dEr, dAr, dRr, dT, dASr, dFr, dSr;
    constant Real dEr = 0;
    constant Real dAr = 0;
    constant Real dRr = 0;
    constant Real dT = 0.1777;
    constant Real dASr = 0;
    constant Real dFr = 0;
    constant Real dSr = -0.0342;
  end ControlInput;
  model new
    Real somepar;
  equation
    somepar = 0.0;
  end new;
  model JetBody_SI
    //Inertial properties
    constant Real m = 4536;
    constant Real Ixx = 35926.5;
    constant Real Iyy = 33940.7;
    constant Real Izz = 67085.5;
    constant Real Ixz = 3418.17;
    //Geometric properties
    constant Real SMI = 0 "Static margin increment";
    constant Real c = 2.14 "Mean aerodynamic chord [m]";
    constant Real b = 10.4 "Wing span [m]";
    constant Real S = 21.5 "Reference Area [m^2]";
    constant Real ARw = 5.02;
    constant Real taperw = 0.507;
    constant Real sweepw = 13 * 0.01745329;
    constant Real ARh = 4;
    constant Real sweeph = 25 * 0.01745329;
    constant Real ARv = 0.64;
    constant Real sweepv = 40 * 0.01745329;
    constant Real lvt = 4.72;
  end JetBody_SI;
  model JetBody
    //Inertial properties
    constant Real m = 10000 "lbm";
    constant Real Ixx = 26498.03 * 32.174 "lbm-ft^2";
    constant Real Iyy = 25033.38 * 32.174;
    constant Real Izz = 49479.73 * 32.174;
    constant Real Ixz = 2521.11 * 32.174;
    //Geometric properties
    constant Real SMI = 0 "Static margin increment";
    constant Real c = 7.02 "Mean aerodynamic chord [ft]";
    constant Real b = 34.12 "Wing span [ft]";
    constant Real S = 231.424 "Reference Area [m^2]";
    constant Real ARw = 5.02;
    constant Real taperw = 0.507;
    constant Real sweepw = 13 * 0.01745329;
    constant Real ARh = 4;
    constant Real sweeph = 25 * 0.01745329;
    constant Real ARv = 0.64;
    constant Real sweepv = 40 * 0.01745329;
    constant Real lvt = 15.486 "vertical tail length [ft]";
  end JetBody;
  package GenericJet
    model Atmosphere
      constant Real rho = 0.002377 * 32.174 "density [slug/ft^3]";
      constant Real P = 2116 "Pressure [lbf/ft^2]";
      constant Real T = 518.69 "temperature [R]";
      constant Real a = 1116.4 "speed of sound [ft/s]";
    end Atmosphere;
    model ControlInput
      //Real dEr, dAr, dRr, dT, dASr, dFr, dSr;
      constant Real dEr = 0;
      constant Real dAr = 0;
      constant Real dRr = 0;
      constant Real dT = 0.1777;
      constant Real dASr = 0;
      constant Real dFr = 0;
      constant Real dSr = -0.0342;
    end ControlInput;
    model JetBody
      //Inertial properties
      constant Real m = 10000 "lbm";
      constant Real Ixx = 26498.03 * 32.174 "lbm-ft^2";
      constant Real Iyy = 25033.38 * 32.174;
      constant Real Izz = 49479.73 * 32.174;
      constant Real Ixz = 2521.11 * 32.174;
      //Geometric properties
      constant Real SMI = 0 "Static margin increment";
      constant Real c = 7.02 "Mean aerodynamic chord [ft]";
      constant Real b = 34.12 "Wing span [ft]";
      constant Real S = 231.424 "Reference Area [m^2]";
      constant Real ARw = 5.02;
      constant Real taperw = 0.507;
      constant Real sweepw = 13 * 0.01745329;
      constant Real ARh = 4;
      constant Real sweeph = 25 * 0.01745329;
      constant Real ARv = 0.64;
      constant Real sweepv = 40 * 0.01745329;
      constant Real lvt = 15.486 "vertical tail length [ft]";
    end JetBody;
    model Jet
      AeroConstants aerodata(alpha = alpha, beta = beta, V = V, p = p, q = q, r = r);
      Atmosphere atmo;
      JetBody body;
      //State variables
      Real u(start = 328.08),v(start = 0),w(start = -10);
      Real x(start = 0),y(start = 0),z(start = 328.08);
      Real p(start = 0),q(start = 0),r(start = 0);
      Real phi(start = 0),theta(start = 0),psi(start = 0);
      //Control variables
      //Real dEr, dAr, dRr, dT, dASr, dFr, dSr;
      //Air relative velocity vector
      Real Reb[3,3],Va[3],Va_e[3],V,alpha(start = 0.1),beta(start = 0),qbar,gb[3];
      //Force and Moment coeffs
      Real CX,CZ,Xb,Yb,Zb,Lb,Mb,Nb;
      annotation(experiment(StartTime = 0.0, StopTime = 100.0, Tolerance = 0.000001));
    equation
      Reb = fun.DCM(phi, theta, psi);
      gb = Reb * {0,0,32.174};
      Va = {u,v,w};
      Va_e = fun.Transpose(Reb) * Va "Velocity vector in inertial frame";
      V = Va * Va;
      alpha = atan(Va[3] / Va[1]);
      beta = asin(Va[2] / V);
      qbar = 0.5 * atmo.rho * V ^ 2;
      CX = -aerodata.CD * cos(alpha) + aerodata.CL * sin(alpha);
      CZ = -aerodata.CD * sin(alpha) - aerodata.CL * cos(alpha);
      body.m * Xb = CX * qbar * body.S;
      body.m * Yb = aerodata.CY * qbar * body.S;
      body.m * Zb = CZ * qbar * body.S;
      Lb = aerodata.Cl * qbar * body.S * body.b;
      Mb = aerodata.Cm * qbar * body.S * body.c;
      Nb = aerodata.Cn * qbar * body.S * body.b;
      der(u) = Xb + gb[1] + r * v - q * w;
      der(v) = Yb + gb[2] - r * u + p * w;
      der(w) = Zb + gb[3] + q * u - p * v;
      der(x) = Va_e[1];
      der(y) = Va_e[2];
      der(z) = Va_e[3];
      body.Ixx * der(p) - body.Ixz * der(r) = (body.Iyy - body.Izz) * q * r + body.Ixz * p * q + Lb;
      body.Iyy * der(q) = (body.Izz - body.Ixx) * p * r + body.Ixz * (r ^ 2 - p ^ 2) + Mb;
      -body.Ixz * der(p) + body.Izz * der(r) = (body.Ixx - body.Iyy) * p * q - body.Ixz * q * r + Nb;
      der(phi) = p + tan(theta) * sin(phi) * q + tan(theta) * cos(phi) * r;
      der(theta) = cos(phi) * q - sin(phi) * r;
      der(psi) * cos(theta) = sin(phi) * q + cos(phi) * r;
    end Jet;
    model AeroConstants
      /*
	06/08/12
	Note:
		1. Consider putting constants in equations section for better readability
		2. Check geometric constants
	*/
      import Modelica.Blocks.Tables.CombiTable1D;
      ControlInput uin;
      Atmosphere atmo;
      JetBody body;
      //Tables used to interpolate data
      CombiTable1D lookup_CL(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CLTable", columns = {2});
      CombiTable1D lookup_CD(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CDTable", columns = {2});
      CombiTable1D lookup_Cm(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CmTable", columns = {2});
      CombiTable1D lookup_CmdE(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CmdETable", columns = {2});
      CombiTable1D lookup_CYBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CYBetaTable", columns = {2});
      CombiTable1D lookup_ClBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "ClBetaTable", columns = {2});
      CombiTable1D lookup_CnBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CnBetaTable", columns = {2});
      CombiTable1D lookup_CldA(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CldATable", columns = {2});
      CombiTable1D lookup_CndA(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CndATable", columns = {2});
      CombiTable1D lookup_CldR(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CldRTable", columns = {2});
      CombiTable1D lookup_CndR(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CndRTable", columns = {2});
      //State variables and other flight conditions
      Real alpha,alpha_deg,beta,beta_deg,V,p,q,r;
      //Control variables
      Real dEr,dAr,dRr,dT,dASr,dFr,dSr;
      //State variables
      //Lift coeff
      Real CLStatic,CLqr,CL;
      constant Real CLdSr = 1.08;
      constant Real CLdEr = 0.5774;
      //Drag coeff
      Real CDStatic,CD;
      //Pitching moment coeff
      Real CmStatic,CmdEr,Cmqr,Cm;
      constant Real CmdSr = -2.291;
      //Side-force coeff
      Real CYBr,CY;
      constant Real CYdAr = -0.00699;
      constant Real CYdRr = 0.1574;
      constant Real CYdASr = 0.0264;
      //Yawing moment coeff
      Real CnBr,Cnpr,Cnrr,CndAr,CndRr,Cn;
      constant Real CndASr = -0.0088;
      //Rolling moment coeff
      Real ClBr,Clpr,Clrr,CldAr,CldRr,Cl;
      constant Real CLar = 5.6575;
      constant Real CldASr = -0.01496;
      //Atmospheric conditions and thrust
      Real Thrust = 0 "N";
      //Geometric Coeffs (change later):
    equation
      uin.dEr = dEr;
      uin.dAr = dAr;
      uin.dRr = dRr;
      uin.dT = dT;
      uin.dASr = dASr;
      uin.dFr = dFr;
      uin.dSr = dSr;
      alpha_deg = fun.r2d(alpha);
      beta_deg = fun.r2d(beta);
      connect(alpha_deg,lookup_CL.u[1]);
      connect(alpha_deg,lookup_CD.u[1]);
      connect(alpha_deg,lookup_Cm.u[1]);
      connect(alpha_deg,lookup_CmdE.u[1]);
      connect(alpha_deg,lookup_CYBeta.u[1]);
      connect(alpha_deg,lookup_ClBeta.u[1]);
      connect(alpha_deg,lookup_CnBeta.u[1]);
      connect(alpha_deg,lookup_CldA.u[1]);
      connect(alpha_deg,lookup_CndA.u[1]);
      connect(alpha_deg,lookup_CldR.u[1]);
      connect(alpha_deg,lookup_CndR.u[1]);
      connect(CLStatic,lookup_CL.y[1]);
      connect(CDStatic,lookup_CD.y[1]);
      connect(CmStatic,lookup_Cm.y[1]);
      connect(CmdEr,lookup_CmdE.y[1]);
      connect(CYBr,lookup_CYBeta.y[1]);
      connect(ClBr,lookup_ClBeta.y[1]);
      connect(CnBr,lookup_CnBeta.y[1]);
      connect(CldAr,lookup_CldA.y[1]);
      connect(CndAr,lookup_CndA.y[1]);
      connect(CldRr,lookup_CldR.y[1]);
      connect(CndRr,lookup_CndR.y[1]);
      CL = CLStatic + CLqr * q + CLdSr * dSr + CLdEr * dEr;
      CLqr = (4.231 * body.c) / (2 * V);
      CD = CDStatic;
      Cmqr = if Cm < -3 then -(18.8 * body.c) / (2 * V) else -0.5;
      Cm = CmStatic - CL * body.SMI + Cmqr * q + CmdSr * dSr + CmdEr * dEr;
      CY = CYBr * beta + CYdRr * dRr + CYdAr * dAr + CYdASr * dASr;
      Cnpr = (CL * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw) * body.b) / (2 * V);
      Cnrr = ((-(2 * body.lvt) / body.b * CnBr - 0.1 * CL ^ 2) * body.b) / (2 * V);
      Cn = CnBr * beta + CndRr * dRr + Cnrr * r + Cnpr * p + CndAr * dAr + CndASr * dASr;
      Clpr = -((CLar * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw)) * body.b) / (2 * V);
      Clrr = ((CL * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw)) * body.b) / (2 * V);
      Cl = ClBr * beta + CldRr * dRr + Clrr * r + Clpr * p + CldAr * dAr + CldASr * dASr;
    end AeroConstants;
  end GenericJet;
  package shadowmaster
    model shadowBody
      //Inertial properties
      constant Real m = 12050 "lbm";
      constant Real Ixx = 15784 * 32.174 "lbm-ft^2";
      constant Real Iyy = 15903 * 32.174;
      constant Real Izz = 34509 * 32.174;
      constant Real Ixz = 0 * 32.174;
      //Geometric properties
      constant Real SMI = 0 "Static margin increment";
      constant Real c = 4.42 "Mean aerodynamic chord [ft]";
      constant Real b = 45.2 "Wing span [ft]";
      constant Real S = 200 "Reference Area [m^2]";
      /* NOT IN USE
	constant Real ARw = 5.02;
  constant Real taperw = 0.507;
  constant Real sweepw = 13 * 0.01745329;
  constant Real ARh = 4;
  constant Real sweeph = 25 * 0.01745329;
  constant Real ARv = 0.64;
  constant Real sweepv = 40 * 0.01745329;
  constant Real lvt = 15.486 "vertical tail length [ft]";
	*/
    end shadowBody;
    model shadowAero_simple
      import Modelica.Blocks.Tables.CombiTable1D;
      import Modelica.Blocks.Tables.CombiTable2D;
      ControlInput uin;
      Atmosphere atmo;
      shadowBody body;
      //Tables used to interpolate data
      //State variables and other flight conditions
      Real alpha,alpha_deg,beta,beta_deg,V,p,q,r;
      //Control variables
      Real dE,dA,dR,dT,dAS,dFL,dFR,dS,dE_deg,dA_deg,dFL_deg,dFR_deg,dFL_norm;
      //State variables
      //Lift coeff
      Real CL,CLalpha,dCLflap;
      constant Real dCLsb = 0;
      constant Real CLde = 0.6;
      constant Real CLq = 19.97;
      //Drag coeff
      Real CD,CD0,CDbeta;
      constant Real CDi = 0.039;
      constant Real CDflap = 0.035;
      constant Real CDsb = 0.028;
      constant Real CDde = 0.035;
      //Pitching moment coeff
      Real Cm;
      constant Real Cmalpha = 0.5;
      constant Real Cmq = -34.2;
      constant Real Cmadot = -8.0;
      //Side-force coeff
      Real CY;
      constant Real CYb = -0.6;
      constant Real CYdr = 0.15;
      constant Real CYp = -0.2;
      constant Real CYr = 0.4;
      //Yawing moment coeff
      Real Cn;
      constant Real Cnb = 0.1;
      constant Real Cnr = -0.18;
      constant Real Cnp = -0.06;
      constant Real Cndr = -0.125;
      constant Real Cnda = -0.001;
      //Rolling moment coeff
      Real Cl,Clb;
      //constant Real Clb = 0.0;
      constant Real Clda = 0.15;
      constant Real Clp = -0.5;
      constant Real Clr = 0.06;
      constant Real Cldr = 0.015;
      //Atmospheric conditions and thrust
      Real Thrust = 0 "lb";
      //Geometric Coeffs (change later):
      annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 0.000001));
      CombiTable1D lookup_CD0(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "CD0", columns = {2});
      CombiTable1D lookup_CDbeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "CDbeta", columns = {2});
      CombiTable1D lookup_CLalpha(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "CLalpha", columns = {2});
      CombiTable1D lookup_dCLflap(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "dCLflap", columns = {2});
      CombiTable2D lookup_Clb(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "Clb", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
      //CombiTable2D lookup_Cmalpha(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable_simple.txt", tableName = "Cmalpha");
    equation
      uin.dE = dE;
      uin.dA = dA;
      uin.dR = dR;
      uin.dT = dT;
      uin.dAS = dAS;
      uin.dFL = dFL;
      uin.dFR = dFR;
      uin.dS = dS;
      dE_deg = fun.r2d(dE);
      dA_deg = fun.r2d(dA);
      dFL_deg = fun.r2d(dFL);
      dFR_deg = fun.r2d(dFR);
      dFL_norm = dFL_deg / 40.0;
      alpha_deg = fun.r2d(alpha);
      beta_deg = fun.r2d(beta);
      connect(alpha,lookup_CD0.u[1]);
      connect(beta,lookup_CDbeta.u[1]);
      connect(alpha,lookup_CLalpha.u[1]);
      connect(dFL_deg,lookup_dCLflap.u[1]);
      connect(beta,lookup_Clb.u1);
      connect(alpha,lookup_Clb.u2);
      beta = lookup_Clb.u1;
      alpha = lookup_Clb.u2;
      connect(alpha,lookup_Cmalpha.u1);
      connect(dE,lookup_Cmalpha.u2);
      connect(CD0,lookup_CD0.y[1]);
      connect(CDbeta,lookup_CDbeta.y[1]);
      connect(CLalpha,lookup_CLalpha.y[1]);
      connect(dCLflap,lookup_dCLflap.y[1]);
      connect(Clb,lookup_Clb.y);
      connect(Cmalpha,lookup_Cmalpha.y);
      Clb = lookup_Clb.y;
      CL = CLalpha + dCLflap + CLde * dE + body.c / (2 * V) * CLq * q;
      CD = CD0 + CDi * CL ^ 2 + CDflap * dFL_norm + CDbeta + CDde * dE;
      Cm = Cmalpha + body.c / (2 * V) * (Cmq + Cmadot * der(alpha));
      CY = CYb * beta + CYdr * dR + body.b / (2 * V) * (CYp * p + CYr * r);
      Cn = Cnb * beta + Cndr * dR + Cnda * dA + body.b / (2 * V) * (Cnr * r + Cnp * p);
      Cl = Clb + Clda * dA + Cldr * dR + body.b / (2 * V) * (Clp * p + Clr * r);
    end shadowAero_simple;
    model GET700
      import Modelica.Blocks.Tables.CombiTable2D;
      extends Engine(Milthrust = 4233.6, idlen1 = 30.0, idlen2 = 60.0, maxn1 = 100.0, maxn2 = 100.0, bpr = 0.0, tsfc = 0.05);
      CombiTable2D lookup_mil(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/GE-T700.txt", tableName = "mil");
      CombiTable2D lookup_idle(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/GE-T700.txt", tableName = "idle");
      //constants
    equation
      connect(M,lookup_mil.u1);
      connect(M,lookup_idle.u1);
      connect(h,lookup_mil.u2);
      connect(h,lookup_idle.u2);
      connect(milfactor,lookup_mil.y);
      connect(idlefactor,lookup_idle.y);
    end GET700;
    model shadowJet
      shadowAero aerodata(tol = 1.0, alpha = alpha, beta = beta, V = V, p = p, q = q, r = r);
      Atmosphere atmo(h = h);
      shadowBody body;
      GET700 engine(h = h, M = M, running = true);
      //State variables
      Real u(start = 100, fixed = true),v(start = 0),w(start = 0);
      Real x(start = 0),y(start = 0),z,h(start = 10001, fixed = true);
      Real p(start = 0),q(start = 0),r(start = 0);
      Real phi(start = 0),theta(start = 1.0, fixed = false),psi(start = 0);
      //Air relative velocity vector
      Real Reb[3,3],Va[3],Va_e[3],V,alpha(start = 1.0, fixed = false),beta(start = 0),qbar,gb[3];
      //Force and Moment coeffs
      Real CX,CZ,Xb,Yb,Zb,Lb,Mb,Nb,M;
      annotation(experiment(StartTime = 0.0, StopTime = 700.0, Tolerance = 0.000001));
      //Debugging variables
      Real theta_deg,alpha_deg,gamma_deg;
    equation
      z = -h;
      Reb = fun.DCM(phi, theta, psi);
      gb = Reb * {0,0,32.174};
      Va = {u,v,w};
      Va_e = fun.Transpose(Reb) * Va "Velocity vector in inertial frame";
      V = sqrt(Va * Va);
      alpha = atan(Va[3] / Va[1]);
      beta = asin(Va[2] / V);
      qbar = 0.5 * atmo.rho * V ^ 2;
      CX = -aerodata.CD * cos(alpha) + aerodata.CL * sin(alpha);
      CZ = -aerodata.CD * sin(alpha) - aerodata.CL * cos(alpha);
      body.m * Xb = CX * qbar * body.S + engine.thrust * 100;
      body.m * Yb = aerodata.CY * qbar * body.S;
      body.m * Zb = CZ * qbar * body.S;
      Lb = aerodata.Cl * qbar * body.S * body.b;
      Mb = aerodata.Cm * qbar * body.S * body.c;
      Nb = aerodata.Cn * qbar * body.S * body.b;
      der(u) = Xb + gb[1] + r * v - q * w;
      der(v) = Yb + gb[2] - r * u + p * w;
      der(w) = Zb + gb[3] + q * u - p * v;
      der(x) = Va_e[1];
      der(y) = Va_e[2];
      der(z) = Va_e[3];
      body.Ixx * der(p) - body.Ixz * der(r) = (body.Iyy - body.Izz) * q * r + body.Ixz * p * q + Lb;
      body.Iyy * der(q) = (body.Izz - body.Ixx) * p * r + body.Ixz * (r ^ 2 - p ^ 2) + Mb;
      -body.Ixz * der(p) + body.Izz * der(r) = (body.Ixx - body.Iyy) * p * q - body.Ixz * q * r + Nb;
      der(phi) = p + tan(theta) * sin(phi) * q + tan(theta) * cos(phi) * r;
      der(theta) = cos(phi) * q - sin(phi) * r;
      der(psi) * cos(theta) = sin(phi) * q + cos(phi) * r;
      M = if atmo.a > 0 then V / atmo.a else 0.3;
      theta_deg = fun.r2d(theta);
      alpha_deg = fun.r2d(alpha);
      gamma_deg = mod(fun.r2d(theta - alpha), 360.0);
    end shadowJet;
    model ControlInput
      import Modelica.Blocks.Tables.CombiTable1D;
      import Modelica.Blocks.Sources.Ramp;
      import Modelica.Blocks.Sources.IntegerStep;
      Ramp ramp(startTime = 200, offset = 0.5, height = 0.5, duration = 200);
      IntegerStep stepup(startTime = 100, height = 1);
      IntegerStep stepdown(startTime = 110, height = 1);
      //CombiTable1D lookup_dE(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/uin.txt", tableName = "dE", columns = {2});
      constant Real dE = -0.1;
      Real dA;
      constant Real dR = 0;
      constant Real dT = 0.1777;
      constant Real dAS = 0;
      constant Real dFL = 0 "left flap pos";
      constant Real dFR = 0 "right flap pos";
      constant Real dS = 0;
    equation
      dA = 0.005 * (stepup.y - stepdown.y);
    end ControlInput;
    model shadowAero
      constant Real tol "tolerance factor";
      import Modelica.Blocks.Tables.CombiTable1D;
      import Modelica.Blocks.Tables.CombiTable2D;
      ControlInput uin;
      shadowBody body;
      //Tables used to interpolate data
      //State variables and other flight conditions
      Real alpha,alpha_deg,beta,beta_deg,V,p,q,r;
      //Control variables
      Real dE,dA,dR,dT,dAS,dFL,dFR,dS,dE_deg,dA_deg,dFL_deg,dFR_deg;
      //State variables
      //Lift coeff
      Real CL,CLStatic,CLq,CLad,CLdF1L,CLdF1R,CLDe;
      //Drag coeff
      Real CD,CDStatic,CdDf1L,CdDf1R,CdDe;
      //Pitching moment coeff
      Real Cm,CmStatic,Cmq,Cmad,CmDe,CmDf1L,CmDf1R;
      //Side-force coeff
      Real CY,Cyb,Cyp;
      //Yawing moment coeff
      Real Cn,Cnb,Cnp,Cnr,CnDf1,CnDa;
      //Rolling moment coeff
      Real Cl,Clb,Clp,Clr,ClDs4,CldF1;
      //Geometric Coeffs (change later):
      annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 0.000001));
      CombiTable1D lookup_CnDf1(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CnDf1", columns = {2});
      CombiTable1D lookup_Cnp(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cnp", columns = {2});
      CombiTable1D lookup_Cnr(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cnr", columns = {2});
      CombiTable2D lookup_CnDa(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CnDa");
      CombiTable1D lookup_Cnb(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cnb", columns = {2});
      CombiTable1D lookup_CmDf1R(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CmDf1R", columns = {2});
      CombiTable1D lookup_CmDf1L(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CmDf1L", columns = {2});
      CombiTable1D lookup_Cmad(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cmadot", columns = {2});
      CombiTable1D lookup_Cmq(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cmq", columns = {2});
      CombiTable1D lookup_CmStatic(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cm_basic", columns = {2});
      CombiTable1D lookup_CldF1(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CldF1", columns = {2});
      CombiTable1D lookup_ClDs4(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "ClDs4", columns = {2});
      CombiTable1D lookup_Clr(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Clr", columns = {2});
      CombiTable1D lookup_CmDe(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CmDe", columns = {2});
      CombiTable1D lookup_Clp(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Clp", columns = {2});
      CombiTable1D lookup_Clb(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Clb", columns = {2});
      CombiTable1D lookup_Cyp(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cyp", columns = {2});
      CombiTable1D lookup_Cyb(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "Cyb", columns = {2});
      CombiTable2D lookup_CdDe(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CdDe");
      CombiTable2D lookup_CdDf1R(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CdDf1R");
      CombiTable2D lookup_CdDf1L(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CdDf1L");
      CombiTable1D lookup_CDStatic(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CD", columns = {2});
      CombiTable1D lookup_CLDe(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLDe", columns = {2});
      CombiTable1D lookup_CLdF1R(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLdF1R", columns = {2});
      CombiTable1D lookup_CLdF1L(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLdF1L", columns = {2});
      CombiTable1D lookup_CLad(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLad", columns = {2});
      CombiTable1D lookup_CLq(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLq", columns = {2});
      CombiTable1D lookup_CLStatic(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/shadowTable.txt", tableName = "CLwbh", columns = {2});
    equation
      uin.dE = dE;
      uin.dA = dA;
      uin.dR = dR;
      uin.dT = dT;
      uin.dAS = dAS;
      uin.dFL = dFL;
      uin.dFR = dFR;
      uin.dS = dS;
      dE_deg = fun.r2d(dE);
      dA_deg = fun.r2d(dA);
      dFL_deg = fun.r2d(dFL);
      dFR_deg = fun.r2d(dFR);
      alpha_deg = fun.r2d(alpha);
      beta_deg = fun.r2d(beta);
      connect(alpha_deg,lookup_CLStatic.u[1]);
      connect(alpha_deg,lookup_CLq.u[1]);
      connect(alpha_deg,lookup_CLad.u[1]);
      connect(dFL_deg,lookup_CLdF1L.u[1]);
      connect(dFR_deg,lookup_CLdF1R.u[1]);
      connect(dE_deg,lookup_CLDe.u[1]);
      connect(alpha_deg,lookup_CDStatic.u[1]);
      connect(alpha_deg,lookup_CdDf1L.u1);
      connect(dFL_deg,lookup_CdDf1L.u2);
      connect(alpha_deg,lookup_CdDf1R.u1);
      connect(dFR_deg,lookup_CdDf1R.u2);
      connect(alpha_deg,lookup_CdDe.u1);
      connect(dE_deg,lookup_CdDe.u2);
      connect(alpha_deg,lookup_Cyb.u[1]);
      connect(alpha_deg,lookup_Cyp.u[1]);
      connect(alpha_deg,lookup_Clb.u[1]);
      connect(alpha_deg,lookup_Clp.u[1]);
      connect(alpha_deg,lookup_Clr.u[1]);
      connect(dA_deg,lookup_ClDs4.u[1]);
      connect(alpha_deg,lookup_CldF1.u[1]);
      connect(alpha_deg,lookup_CmStatic.u[1]);
      connect(alpha_deg,lookup_Cmq.u[1]);
      connect(alpha_deg,lookup_Cmad.u[1]);
      connect(dE_deg,lookup_CmDe.u[1]);
      connect(dFL_deg,lookup_CmDf1L.u[1]);
      connect(dFR_deg,lookup_CmDf1R.u[1]);
      connect(alpha_deg,lookup_Cnb.u[1]);
      connect(alpha_deg,lookup_Cnp.u[1]);
      connect(alpha_deg,lookup_Cnr.u[1]);
      connect(alpha_deg,lookup_CnDf1.u[1]);
      connect(alpha_deg,lookup_CnDa.u1);
      connect(dA_deg,lookup_CnDa.u2);
      connect(CLStatic,lookup_CLStatic.y[1]);
      connect(CLq,lookup_CLq.y[1]);
      connect(CLad,lookup_CLad.y[1]);
      connect(CLdF1L,lookup_CLdF1L.y[1]);
      connect(CLdF1R,lookup_CLdF1R.y[1]);
      connect(CLDe,lookup_CLDe.y[1]);
      connect(CDStatic,lookup_CDStatic.y[1]);
      connect(CdDf1L,lookup_CdDf1L.y);
      connect(CdDf1R,lookup_CdDf1R.y);
      connect(CdDe,lookup_CdDe.y);
      connect(Cyb,lookup_Cyb.y[1]);
      connect(Cyp,lookup_Cyp.y[1]);
      connect(Clb,lookup_Clb.y[1]);
      connect(Clp,lookup_Clp.y[1]);
      connect(Clr,lookup_Clr.y[1]);
      connect(ClDs4,lookup_ClDs4.y[1]);
      connect(CldF1,lookup_CldF1.y[1]);
      connect(CmStatic,lookup_CmStatic.y[1]);
      connect(Cmq,lookup_Cmq.y[1]);
      connect(Cmad,lookup_Cmad.y[1]);
      connect(CmDe,lookup_CmDe.y[1]);
      connect(CmDf1L,lookup_CmDf1L.y[1]);
      connect(CmDf1R,lookup_CmDf1R.y[1]);
      connect(Cnb,lookup_Cnb.y[1]);
      connect(Cnp,lookup_Cnp.y[1]);
      connect(Cnr,lookup_Cnr.y[1]);
      connect(CnDf1,lookup_CnDf1.y[1]);
      connect(CnDa,lookup_CnDa.y);
      CL = if abs(CL) > 1.5 then tol * (CLStatic + body.c / (2 * V) * (CLq * fun.r2d(q) + CLad * fun.r2d(der(alpha))) + CLdF1L + CLdF1R + CLDe) else CLStatic + body.c / (2 * V) * (CLq * fun.r2d(q) + CLad * der(alpha_deg)) + CLdF1L + CLdF1R + CLDe;
      CD = CDStatic + CdDf1L + CdDf1R + CdDe;
      Cm = if abs(Cm) > 0.6 then tol * (CmStatic + body.c / (2 * V) * (Cmq * fun.r2d(q) + Cmad * fun.r2d(der(alpha))) + CmDe + CmDf1L + CmDf1R) else CmStatic + body.c / (2 * V) * (Cmq * fun.r2d(q) + Cmad * der(alpha_deg)) + CmDe + CmDf1L + CmDf1R;
      CY = Cyb * beta_deg + body.b / (2 * V) * Cyp * fun.r2d(p);
      Cn = Cnb * beta_deg + body.b / (2 * V) * (Cnp * fun.r2d(p) + Cnr * fun.r2d(r)) + CnDf1 * (CdDf1R - CdDf1L) + CnDa;
      Cl = Clb * beta_deg + body.b / (2 * V) * Clp * fun.r2d(p) + ClDs4 + CldF1 * (CLdF1R - CLdF1L);
    end shadowAero;
  end shadowmaster;
  model Jet
    AeroConstants aerodata(alpha = alpha, beta = beta, V = V, p = p, q = q, r = r);
    Atmosphere atmo;
    JetBody body;
    //State variables
    Real u(start = 328.08),v(start = 0),w(start = -10);
    Real x(start = 0),y(start = 0),z(start = 328.08);
    Real p(start = 0),q(start = 0),r(start = 0);
    Real phi(start = 0),theta(start = 0),psi(start = 0);
    //Control variables
    //Real dEr, dAr, dRr, dT, dASr, dFr, dSr;
    //Air relative velocity vector
    Real Reb[3,3],Va[3],Va_e[3],V,alpha(start = 0.1),beta(start = 0),qbar,gb[3];
    //Force and Moment coeffs
    Real CX,CZ,Xb,Yb,Zb,Lb,Mb,Nb;
    annotation(experiment(StartTime = 0.0, StopTime = 100.0, Tolerance = 0.000001));
  equation
    Reb = fun.DCM(phi, theta, psi);
    gb = Reb * {0,0,32.174};
    Va = {u,v,w};
    Va_e = fun.Transpose(Reb) * Va "Velocity vector in inertial frame";
    V = Va * Va;
    alpha = atan(Va[3] / Va[1]);
    beta = asin(Va[2] / V);
    qbar = 0.5 * atmo.rho * V ^ 2;
    CX = -aerodata.CD * cos(alpha) + aerodata.CL * sin(alpha);
    CZ = -aerodata.CD * sin(alpha) - aerodata.CL * cos(alpha);
    body.m * Xb = CX * qbar * body.S;
    body.m * Yb = aerodata.CY * qbar * body.S;
    body.m * Zb = CZ * qbar * body.S;
    Lb = aerodata.Cl * qbar * body.S * body.b;
    Mb = aerodata.Cm * qbar * body.S * body.c;
    Nb = aerodata.Cn * qbar * body.S * body.b;
    der(u) = Xb + gb[1] + r * v - q * w;
    der(v) = Yb + gb[2] - r * u + p * w;
    der(w) = Zb + gb[3] + q * u - p * v;
    der(x) = Va_e[1];
    der(y) = Va_e[2];
    der(z) = Va_e[3];
    body.Ixx * der(p) - body.Ixz * der(r) = (body.Iyy - body.Izz) * q * r + body.Ixz * p * q + Lb;
    body.Iyy * der(q) = (body.Izz - body.Ixx) * p * r + body.Ixz * (r ^ 2 - p ^ 2) + Mb;
    -body.Ixz * der(p) + body.Izz * der(r) = (body.Ixx - body.Iyy) * p * q - body.Ixz * q * r + Nb;
    der(phi) = p + tan(theta) * sin(phi) * q + tan(theta) * cos(phi) * r;
    der(theta) = cos(phi) * q - sin(phi) * r;
    der(psi) * cos(theta) = sin(phi) * q + cos(phi) * r;
  end Jet;
  model AeroConstants
    /*
	06/08/12
	Note:
		1. Consider putting constants in equations section for better readability
		2. Check geometric constants
	*/
    import Modelica.Blocks.Tables.CombiTable1D;
    ControlInput uin;
    Atmosphere atmo;
    JetBody body;
    //Tables used to interpolate data
    CombiTable1D lookup_CL(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CLTable", columns = {2});
    CombiTable1D lookup_CD(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CDTable", columns = {2});
    CombiTable1D lookup_Cm(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CmTable", columns = {2});
    CombiTable1D lookup_CmdE(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CmdETable", columns = {2});
    CombiTable1D lookup_CYBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CYBetaTable", columns = {2});
    CombiTable1D lookup_ClBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "ClBetaTable", columns = {2});
    CombiTable1D lookup_CnBeta(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CnBetaTable", columns = {2});
    CombiTable1D lookup_CldA(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CldATable", columns = {2});
    CombiTable1D lookup_CndA(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CndATable", columns = {2});
    CombiTable1D lookup_CldR(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CldRTable", columns = {2});
    CombiTable1D lookup_CndR(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/aeroTable.txt", tableName = "CndRTable", columns = {2});
    //State variables and other flight conditions
    Real alpha,alpha_deg,beta,beta_deg,V,p,q,r;
    //Control variables
    Real dEr,dAr,dRr,dT,dASr,dFr,dSr;
    //State variables
    //Lift coeff
    Real CLStatic,CLqr,CL;
    constant Real CLdSr = 1.08;
    constant Real CLdEr = 0.5774;
    //Drag coeff
    Real CDStatic,CD;
    //Pitching moment coeff
    Real CmStatic,CmdEr,Cmqr,Cm;
    constant Real CmdSr = -2.291;
    //Side-force coeff
    Real CYBr,CY;
    constant Real CYdAr = -0.00699;
    constant Real CYdRr = 0.1574;
    constant Real CYdASr = 0.0264;
    //Yawing moment coeff
    Real CnBr,Cnpr,Cnrr,CndAr,CndRr,Cn;
    constant Real CndASr = -0.0088;
    //Rolling moment coeff
    Real ClBr,Clpr,Clrr,CldAr,CldRr,Cl;
    constant Real CLar = 5.6575;
    constant Real CldASr = -0.01496;
    //Atmospheric conditions and thrust
    Real Thrust = 0 "N";
    //Geometric Coeffs (change later):
  equation
    uin.dEr = dEr;
    uin.dAr = dAr;
    uin.dRr = dRr;
    uin.dT = dT;
    uin.dASr = dASr;
    uin.dFr = dFr;
    uin.dSr = dSr;
    alpha_deg = fun.r2d(alpha);
    beta_deg = fun.r2d(beta);
    connect(alpha_deg,lookup_CL.u[1]);
    connect(alpha_deg,lookup_CD.u[1]);
    connect(alpha_deg,lookup_Cm.u[1]);
    connect(alpha_deg,lookup_CmdE.u[1]);
    connect(alpha_deg,lookup_CYBeta.u[1]);
    connect(alpha_deg,lookup_ClBeta.u[1]);
    connect(alpha_deg,lookup_CnBeta.u[1]);
    connect(alpha_deg,lookup_CldA.u[1]);
    connect(alpha_deg,lookup_CndA.u[1]);
    connect(alpha_deg,lookup_CldR.u[1]);
    connect(alpha_deg,lookup_CndR.u[1]);
    connect(CLStatic,lookup_CL.y[1]);
    connect(CDStatic,lookup_CD.y[1]);
    connect(CmStatic,lookup_Cm.y[1]);
    connect(CmdEr,lookup_CmdE.y[1]);
    connect(CYBr,lookup_CYBeta.y[1]);
    connect(ClBr,lookup_ClBeta.y[1]);
    connect(CnBr,lookup_CnBeta.y[1]);
    connect(CldAr,lookup_CldA.y[1]);
    connect(CndAr,lookup_CndA.y[1]);
    connect(CldRr,lookup_CldR.y[1]);
    connect(CndRr,lookup_CndR.y[1]);
    CL = CLStatic + CLqr * q + CLdSr * dSr + CLdEr * dEr;
    CLqr = (4.231 * body.c) / (2 * V);
    CD = CDStatic;
    Cmqr = if Cm < -3 then -(18.8 * body.c) / (2 * V) else -0.5;
    Cm = CmStatic - CL * body.SMI + Cmqr * q + CmdSr * dSr + CmdEr * dEr;
    CY = CYBr * beta + CYdRr * dRr + CYdAr * dAr + CYdASr * dASr;
    Cnpr = (CL * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw) * body.b) / (2 * V);
    Cnrr = ((-(2 * body.lvt) / body.b * CnBr - 0.1 * CL ^ 2) * body.b) / (2 * V);
    Cn = CnBr * beta + CndRr * dRr + Cnrr * r + Cnpr * p + CndAr * dAr + CndASr * dASr;
    Clpr = -((CLar * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw)) * body.b) / (2 * V);
    Clrr = ((CL * (1 + 3 * body.taperw)) / (12 * (1 + body.taperw)) * body.b) / (2 * V);
    Cl = ClBr * beta + CldRr * dRr + Clrr * r + Clpr * p + CldAr * dAr + CldASr * dASr;
  end AeroConstants;
  model Atmosphere
    import Modelica.Blocks.Tables.CombiTable1D;
    CombiTable1D lookup_T(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/atmosphere.txt", tableName = "atmo", columns = {2});
    CombiTable1D lookup_a(tableOnFile = true, fileName = "/hsl/homes/wu46/Projects/JetModel/tables/atmosphere.txt", tableName = "atmo", columns = {3});
    Real h,h_geo,betap,betar,P,rho,T,a;
    Integer k;
    constant Real R = 6367435 * 3.2808399;
    constant Real Pres = 2116 "Pressure [lbf/ft^2]";
    constant Real Dens = 0.002377 * 32.174 "density [slug/ft^3]";
    constant Real Z[10] = 3.2808388 * {-10,0,2500,5000,10000,11100,15000,20000,47400,51000} "ft";
    constant Real ppo[10] = {1,1,0.737,0.533,0.262,0.221,0.12,0.055,0.0011,0.0007};
    constant Real rro[10] = {1,1,0.781,0.601,0.338,0.293,0.159,0.073,0.0011,0.0007};
    function findz
      input Real Z[10];
      input Real h;
      output Integer k;
    algorithm
      for n in 2:1:10 loop
              if h <= Z[n] then 
                k:=n;
        break;

        else 
        end if;

      end for;
    end findz;
  equation
    k = findz(Z, h);
    h_geo = (R * h) / (R + h);
    betap = log(ppo[k]) / ppo[k - 1] / (Z[k] - Z[k - 1]);
    betar = log(rro[k]) / rro[k - 1] / (Z[k] - Z[k - 1]);
    P = Pres * ppo[k - 1] * exp(betap * (h - Z[k - 1]));
    rho = Dens * rro[k - 1] * exp(betar * (h - Z[k - 1]));
    connect(h,lookup_T.u[1]);
    connect(h,lookup_a.u[1]);
    connect(T,lookup_T.y[1]);
    connect(a,lookup_a.y[1]);
  end Atmosphere;
  model Engine
    ControlInput uin;
    Real M,h;
    Boolean running;
    //inputs for lookup table
    Real milthrust,idlethrust,Milthrust,milfactor,idlefactor,thrust;
    constant Real idlen1,idlen2,maxn1,maxn2,bpr,tsfc;
    Real n2,n2norm,n2factor;
  equation
    idlethrust = Milthrust * idlefactor;
    milthrust = (Milthrust - idlethrust) * milfactor;
    thrust = if running then idlethrust + milthrust * n2norm * n2norm else 0.0;
    n2 = idlen2 + uin.dT * n2factor;
    n2norm = (n2 - idlen2) / n2factor;
    n2factor = maxn2 - idlen2;
  end Engine;
end JetPack;

