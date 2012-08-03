within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"
  import MB=MultiBodyOmc;
  extends MB.Parts.Body(
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r_0(start={0,0,-10000}, fixed=true),
    v_0(start={10,0,0}, fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    angles_start={1,1,1},
    animation=false);
end Airframe;

model AirframeInitGlide
  extends Airframe(
    // solve for velocity and attitude
    v_0(start={10,0,0}, fixed=false),
    angles_fixed = false,
    // given, no angular acceleration
    z_0_fixed = true,
    animation=false);
end AirframeInitGlide;

// vim:ts=2:sw=2:expandtab:
