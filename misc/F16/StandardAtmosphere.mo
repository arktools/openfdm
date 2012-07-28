model StandardAtmosphere
  constant Real rho0 = 2.377e-3 "sea-level density";
  input Real alt "altitude";
  input Real vt "velocity";
  output Real t "temperature, K";
  output Real mach "mach number";
  output Real qbar "dynamics pressure";
  output Real rho "air density";
  output Real ps "static pressure";
protected
  Real tfac "temperature factor";
equation
  tfac = 1 - 0.703e-5 * alt;
  if (alt >= 35000.0) then
    t = 390.0;
  else
    t = 519.0 * tfac;
  end if;
  rho = rho0 * (tfac^4.14);
  mach = vt/sqrt(1.4*1716.3*t);
  qbar = 0.5*rho*vt*vt;
  ps = 1715.0 * rho * t;
end StandardAtmosphere;

// vim:sw=2:ts=2:expandtab:
