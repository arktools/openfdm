within MultiBodyOmc;

package Test

  model OrientationTest
    inner World world(enableAnimation=false);
    Forces.WorldForceAndTorque aero(animation=false);      
    Parts.Body body(
      m=1,
      I_11=1,
      I_22=1,
      I_33=1,
      r_0(start={0,0,0}, fixed=true),
      v_0(start={1,0,0}, fixed=true),
      angles_fixed=true,
      w_0_fixed=true,
      angles_start={1,2,3},
      animation=false,
      useQuaternions=false
      );

    //Interfaces.Frame frame_a;
  equation
    connect(body.frame_a,aero.frame_b);
    aero.force = 0*ones(3);
    aero.torque =0*ones(3);

    /*connect(frame_a,aero.frame_b);*/

    /*r_0 = frame_a.r_0;*/
    /*der(r_0) = v_0;*/
    /*der(v_0) = a_0;*/
    /*frame_a.f + m*a_0 = zeros(3);*/

    /*w_a = Frames.angularVelocity2(frame_a.R);*/
    /*der(I*w_a) + frame_a.t = zeros(3);*/

  end OrientationTest;

end Test;

// vim:ts=2:sw=2:expandtab
