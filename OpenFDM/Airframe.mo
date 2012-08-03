within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"
  import MB=MultiBodyOmc;
  extends MB.Parts.Body(
    animation=false,
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates);
end Airframe;

model AirframeInitGlide
  extends Airframe(
    // solve for velocity and attitude
    v_0(start={10,0,0}, fixed=false),
    angles_start = {0,-0.1,0}, // guess
    angles_fixed = false,
    // given
    r_0(start={0,0,-10000}, fixed=true),
    w_0_fixed = true,
    w_0_start = {0,0,0},
    z_0_fixed = true,
    z_0_start = {0,0,0});
end AirframeInitGlide;

// vim:ts=2:sw=2:expandtab:
