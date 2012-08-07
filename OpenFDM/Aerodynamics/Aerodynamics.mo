within OpenFDM.Aerodynamics; 

partial model AerodynamicForceMoment "partial force model that computes aerodynamic relavent properties, still required to define F_b, M_B"
  extends Parts.ForceMoment;
  import SI=Modelica.SIunits;
  outer World.WorldBase world;
  parameter SI.Velocity vtTol = 0.01 "when to ignore aerodynamics";
  SI.Velocity vR_b[3] "relative air velocity resolved in body frame";
  SI.Acceleration aR_b[3] "relative air acceleration resolved in body frame";
  SI.Velocity vt "true air velocity";
  SI.Acceleration vtDot "true air velocity derivative";
  SI.Pressure qBar "dynamics pressure";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "derivative of angle of attack";
  SI.Angle beta "side slip angle";
  SI.AngularVelocity betaDot "derivative of side slip angle";
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
    betaDot = (aR_b[2]*vt - vR_b[2]*vtDot) /
      vt*sqrt(vR_b[1]^2 + vR_b[3]^2);
  else
    vtDot = 0;
    beta = 0;
    betaDot = 0;
  end if;
end AerodynamicForceMoment;

// vim:ts=2:sw=2:expandtab:
