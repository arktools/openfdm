within OpenFDM.Aerodynamics.Examples;

model BodyFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.BodyFrame.ForceAndTorque aerodynamics(
    coefs(CX=0,CY=0,CZ=0,Cl=0,Cm=0,Cn=0,
      s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end BodyFrameAeroObject;

model WindFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.WindFrame.ForceAndTorque aerodynamics(
    coefs(CD=0,CC=0,CL=0,Cl=0,Cm=0,Cn=0,
      s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end WindFrameAeroObject;

model StabilityFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.StabilityFrame.ForceAndTorque aerodynamics(
    coefs(CD=0,CY=0,CL=0,Cl=0,Cm=0,Cn=0,
      s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end StabilityFrameAeroObject;

model ForceAndTorqueEx
  import MB=MultiBodyOmc;
  inner MB.World world(n={0,0,1});
  BodyFrameAeroObject body_b;
  WindFrameAeroObject body_w;
  StabilityFrameAeroObject body_s;
end ForceAndTorqueEx;

// vim:ts=2:sw=2:expandtab:
