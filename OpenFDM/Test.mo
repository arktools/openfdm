within OpenFDM;

package Test

  model FlatEarth6DOF
    inner World.Earth world;
    import Aero=OpenFDM.Aerodynamics.StabilityFrame;
    Aircraft.FlatEarth6DOF aircraft(
      v_n(start={10,0,0}));
    Aero.SimpleForceAndTorque aerodynamics(
      qBar=aircraft.qBar,
      alpha=aircraft.alpha,
      alphaDot=aircraft.alphaDot,
      beta=aircraft.beta,
      betaDot=aircraft.betaDot,
      vt=aircraft.vt,
      vtDot=aircraft.vtDot,
      p=aircraft.w_b[1],
      q=aircraft.w_b[2],
      r=aircraft.w_b[3],
      // controls
      aileron_deg = 0,
      elevator_deg = 0,
      rudder_deg = 0,
      flap_deg = 0,
      // lift
      CL0 = 0.1,
      CLa = 0.1/20.0,
      // drag
      CD0 = 0.01,
      CDCL = 0.01,
      // side force
      CYb = 0.01,
      // roll moment
      Clp = -0.1,
      Clda = 0.1,
      // pitch moment
      Cmq = -0.1,
      Cma = -0.1,
      Cmde = 0.1,
      // yaw moment
      Cnb = 0.1,
      Cnr = -0.1,
      Cndr = 0.1,
      //Stall angle
      alphaStall_deg = 30.0,
      coefs(s=1, b=1, cBar=1));
  equation
    connect(aerodynamics.frame,aircraft.frame);
  end FlatEarth6DOF;

end Test;

// vim:ts=2:sw=2:expandtab
