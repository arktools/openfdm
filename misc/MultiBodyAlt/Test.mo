within OpenFDM;

package Test

  model FlatEarth6DOF
    inner World.Earth world;
    import Aero=OpenFDM.Aerodynamics.StabilityFrame;
    Aircraft.FlatEarth6DOF aircraft(
      phi(fixed=true, start=0),
      theta(fixed=false, start=1),
      psi(fixed=true, start=0),
      v_n(fixed=true, start={10,0,0}));
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
    Propulsion.ConstantThrust engine(thrust=1);
  initial equation
    der(aircraft.w_b) = {0,0,0};
  equation
    connect(aerodynamics.frame,aircraft.frame);
    connect(engine.frame,aircraft.frame);
  end FlatEarth6DOF;

  model FreeBodyTest
    import MB=MultiBodyOmc;
    inner MB.World world(enableAnimation=false,n={0,0,1});
    MB.Forces.WorldForceAndTorque extFT(animation=false);      
    MB.Parts.Body body(
      m=1,
      I_11=1,
      I_22=1,
      I_33=1,
      r_0(start={0,0,0}, fixed=true),
      v_0(start={1,0,0}, fixed=true),
      angles_fixed=true,
      w_0_fixed=true,
      angles_start={1,2,3},
      animation=false);
  equation
    connect(body.frame_a,extFT.frame_b);
    extFT.force = 0*ones(3);
    extFT.torque =0*ones(3);
  end FreeBodyTest;

end Test;

// vim:ts=2:sw=2:expandtab


// vim:ts=2:sw=2:expandtab
