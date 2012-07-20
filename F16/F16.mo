model F16

  // control variables
  /*
  input Real throttle(start = init.throttle);
  input Real elevator;
  input Real aileron;
  input Real rudder;
  */

  
  Airframe airframe(
    vt(start=init.vt),
    alpha(start=init.alpha),
    beta(start=init.beta),
    phi(start=init.phi),
    theta(start=init.theta),
    psi(start=init.psi),
    p(start=init.p),
    q(start=init.q),
    r(start=init.r),
    posNorth(start=init.posNorth),
    posEast(start=init.posEast),
    alt(start=init.alt)
  );
  //F100_PW_200 engine(power(start=init.power));
  NullEngine engine;
  StandardAtmosphere atmosphere;
  Aerodynamics aerodynamics;
  //AerodynamicsNull aerodynamics;
  Controls controls;
  StateInit init;

equation
  
  // propulsion
  connect(controls.throttle,engine.throttle);
  connect(atmosphere.mach,engine.mach);
  connect(airframe.alt,engine.alt);
  connect(engine.thrust,airframe.thrust); 

  // atmosphere
  connect(airframe.alt,atmosphere.alt);
  connect(airframe.vt,atmosphere.vt);
  connect(atmosphere.mach,airframe.mach);
  connect(atmosphere.qbar,airframe.qbar);

  // aerodynamics
  connect(aerodynamics.cx,airframe.cx);
  connect(aerodynamics.cy,airframe.cy);
  connect(aerodynamics.cz,airframe.cz);
  connect(aerodynamics.cl,airframe.cl);
  connect(aerodynamics.cm,airframe.cm);
  connect(aerodynamics.cn,airframe.cn);
  connect(airframe.alpha, aerodynamics.alpha);
  connect(airframe.beta, aerodynamics.beta);
  connect(airframe.vt, aerodynamics.vt);
  connect(airframe.p, aerodynamics.p);
  connect(airframe.q, aerodynamics.q);
  connect(airframe.r, aerodynamics.r);
  connect(airframe.b, aerodynamics.b);
  connect(airframe.cbar, aerodynamics.cbar);
  connect(airframe.xcg, aerodynamics.xcg);
  connect(airframe.xcgr, aerodynamics.xcgr);
  connect(controls.elevator, aerodynamics.elevator);
  connect(controls.aileron, aerodynamics.aileron);
  connect(controls.rudder, aerodynamics.rudder);
  // control
  connect(controls.throttle,airframe.throttle);
  connect(controls.elevator,airframe.elevator);
  connect(controls.aileron,airframe.aileron);
  connect(controls.rudder,airframe.rudder);
  //connect(init.throttle, throttle);

end F16;

// vim:sw=2:ts=2:expandtab:
