within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"
  import MB=MultiBodyOmc;
  import SI=Modelica.SIunits;
  parameter Boolean a_0_fixed = false;
  parameter Boolean v_0_fixed = false;
  parameter Boolean r_0_fixed = false;
  parameter SI.Position r_0_start[3] = {0,0,-1000000};
  parameter SI.Velocity v_0_start[3] = {20,0,0};
  parameter SI.Acceleration a_0_start[3] = {0,0,0};
  extends MB.Parts.Body(
    useQuaternions=false,
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r_0(start=r_0_start, each fixed=r_0_fixed),
    v_0(start=v_0_start, each fixed=v_0_fixed),
    a_0(start=a_0_start, each fixed=a_0_fixed),
    angles_fixed=false, 
    w_0_fixed=true,
    angles_start={0,0,0},
    animation=false);
end Airframe;

model AirframeInitGlide
  extends Airframe(
    // solve for velocity and attitude
    v_0_fixed = false,
    angles_fixed = false,
    // given, no angular/rot acceleration
    //z_0_fixed = true,
    a_0_fixed = true,
    animation=false);
end AirframeInitGlide;

model WorldNED
  import MB=MultiBodyOmc;
  extends MB.World(enableAnimation=false,n={0,0,1});
end WorldNED;

// vim:ts=2:sw=2:expandtab:
