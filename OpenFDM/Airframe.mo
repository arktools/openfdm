within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"
  import MB=MultiBodyOmc;
  extends MB.Parts.Body(
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates);
end Airframe;

// vim:ts=2:sw=2:expandtab:
