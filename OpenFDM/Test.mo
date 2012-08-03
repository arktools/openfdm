within OpenFDM;

package Test

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
