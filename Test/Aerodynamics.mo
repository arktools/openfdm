within Test; 

partial model Aerodynamics "partial force model that computes aerodynamic relavent properties, still required to define F_b, M_B"
  extends ForceMoment;
  outer World world;
  parameter Real vtTol = 0.01 "when to ignore aerodynamics";
  Real vR_b[3] "relative air velocity resolved in body frame";
  Real aR_b[3] "relative air acceleration resolved in body frame";
  Real vt "true air velocity";
  Real vtDot "true air velocity derivative";
  Real qBar "dynamics pressure";
  Real alpha "angle of attack";
  Real alphaDot "derivative of angle of attack";
  Real beta "side slip angle";
  Real betaDot "derivative of side slip angle";
equation
  vR_b = fA.v_b - fA.C_br*world.wind_r(fA.r_r);
  aR_b = der(vR_b);
  vt = sqrt(vR_b*vR_b);
  qBar = 0.5*world.rho(fA.r_r)*vt^2;
  if (vR_b[1] > vtTol) then
    alpha = atan2(vR_b[3],vR_b[1]);
    // omc doesn't like der(beta), setting manually
    alphaDot = (vR_b[1]*aR_b[3]-vR_b[3]*aR_b[1])/
      (vR_b[1]^2 + vR_b[3]^2); //stevens & lewis pg 78
  else
    alpha = 0;
    alphaDot = 0;
  end if;
  if (vt > vtTol) then
    // omc doesn't like der(vt), setting manually
    vtDot = (vR_b[1]*aR_b[1] + 
          vR_b[2]*aR_b[2] +
          vR_b[3]*aR_b[3])/vt;
    beta = asin(vR_b[2]/vt);
    // omc doesn't like der(beta), setting manually
    betaDot = (aR_b[2]*vt - aR_b[2]*vtDot) /
      vt*sqrt(vR_b[1]^2 + vR_b[3]^2);
  else
    vtDot = 0;
    beta = 0;
    betaDot = 0;
  end if;
end Aerodynamics;

// vim:ts=2:sw=2:expandtab:
