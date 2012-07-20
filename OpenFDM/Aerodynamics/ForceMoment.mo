within OpenFDM.Aerodynamics;

model ForceMoment
  import OpenFDM.*;
 
  AircraftState state(alpha = 1, alphaDot = 1, p = 1, q = 1, r = 1);
  input TableConnector coef;
  DatcomTable_Citation table;
  constant Real V = 1;
  constant Real bref = 1;
  constant Real cbar = 1;
  constant Real q = 1;
  constant Real S = 1;
  Real CL_static;
  Real CL_dyn;
  Real CL;
  Real CD_static;
  Real CD_dyn;
  Real CD;
  Real Cy_static;
  Real Cy_dyn;
  Real Cy;
  Real Cl_static;
  Real Cl_dyn;
  Real Cl;
  Real Cm_static;
  Real Cm_dyn;
  Real Cm;
  Real Cn_static;
  Real Cn_dyn;
  Real Cn;
  Real L;
  Real D;
  Real Y;
  Real P;
  Real Q;
  Real R;
equation
  connect(table.state,state);
  connect(table.coef,coef);
  CL_static = (coef.CL_FLAPS+coef.CL_AILERONS+coef.CL_TOTAL)+(coef.CLA_FLAPS+coef.CLA_AILERONS+coef.CLA_TOTAL)*state.alpha;
  CL_dyn = (coef.CLAD_FLAPS+coef.CLAD_AILERONS+coef.CLAD_TOTAL)*state.alphaDot*(cbar/2/V);
  CL = CL_static+CL_dyn;
  CD_static = (coef.CD_FLAPS+coef.CD_AILERONS+coef.CD_TOTAL);
  CD_dyn = 0;
  CD = CD_static+CD_dyn;
  Cy_static = (coef.CYB_FLAPS+coef.CYB_AILERONS+coef.CYB_TOTAL);
  Cy_dyn = (coef.CYP_FLAPS+coef.CYP_AILERONS+coef.CYP_TOTAL)*state.p*(bref/2/V);
  Cy = Cy_static+Cy_dyn;
  Cl_static = (coef.CLB_FLAPS+coef.CLB_AILERONS+coef.CLB_TOTAL);
  Cl_dyn = ((coef.CLP_FLAPS+coef.CLP_AILERONS+coef.CLP_TOTAL)*state.p+(coef.CLQ_FLAPS+coef.CLQ_AILERONS+coef.CLQ_TOTAL)*state.q+(coef.CLR_FLAPS+coef.CLR_AILERONS+coef.CLR_TOTAL)*state.r)*bref/2/V;
  Cl = Cl_static+Cl_dyn;
  Cm_static = (coef.CM_FLAPS+coef.CM_AILERONS+coef.CM_TOTAL)+(coef.CMA_FLAPS+coef.CMA_AILERONS+coef.CMA_TOTAL)*state.alpha;
  Cm_dyn = ((coef.CMQ_FLAPS+coef.CMQ_AILERONS+coef.CMQ_TOTAL)*state.q+(coef.CMAD_FLAPS+coef.CMAD_AILERONS+coef.CMAD_TOTAL)*state.alphaDot)*cbar/2/V;
  Cm = Cm_static+Cm_dyn;
  Cn_static = coef.CNB_FLAPS+coef.CNB_AILERONS+coef.CNB_TOTAL;
  Cn_dyn = ((coef.CNP_FLAPS+coef.CNP_AILERONS+coef.CNP_TOTAL)*state.p+(coef.CNR_FLAPS+coef.CNR_AILERONS+coef.CNR_TOTAL)*state.r)*bref/2/V;
  Cn = Cn_static+Cn_dyn;
  
  L = q*S*CL;
  D = q*S*CD;
  Y = q*S*Cy;
  P = q*S*cbar*Cl;
  Q = q*S*cbar*Cm;
  R = q*S*cbar*Cn;
end ForceMoment;
