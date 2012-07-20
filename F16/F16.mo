model F16

  // control variables
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  Airframe airframe;
  F100_PW_200 engine;
  StandardAtmosphere atmosphere;
  Aerodynamics aerodynamics;

equation

  // propulsion
  connect(throttle,engine.throttle);
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
  connect(elevator, aerodynamics.elevator);
  connect(aileron, aerodynamics.aileron);
  connect(rudder, aerodynamics.rudder);

  // control
  connect(throttle,airframe.throttle);
  connect(elevator,airframe.elevator);
  connect(aileron,airframe.aileron);
  connect(rudder,airframe.rudder);

end F16;

// vim:sw=2:ts=2:expandtab:
