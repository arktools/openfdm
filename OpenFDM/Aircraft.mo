within OpenFDM;

package Aircraft

  import SI=Modelica.SIunits;
  import NonSI=Modelica.SIunits.Conversions.NonSIunits;

  model Base
    Connectors.Frame frame;
    outer World.Base world;
    Real aileron_deg = 0;
    Real elevator_deg = 0;
    Real rudder_deg = 0;
    Real flap_deg = 0;
    parameter SI.Mass m = 1 "mass";
    parameter SI.MomentOfInertia J[3,3] = identity(3);
    SI.Velocity[3] v_b "velocity in body frame";
    SI.Acceleration[3] a_b "acceleration in body frame";
    SI.AngularVelocity[3] w_b 
      "angular velocity in body frame";
    SI.Angle phi "euler roll angle";
    SI.Angle theta "euler pitch angle";
    SI.Angle psi "euler heading angle";
    SI.Velocity v_n[3] "velocity in NED frame";
    SI.Position asl "altitude above sea level";
    SI.Angle lat "latitude";
    SI.Angle lng "longitude";
    Real C_nb[3,3];
    Real C_bn[3,3];
    SI.AngularMomentum H[3] "angular momentum";
    SI.Force F_b[3] "force in body frame";
    SI.Torque M_b[3] "moment in body frame";

    // aerodynamic properties
    SI.Velocity vt "true airspeed";
    SI.Velocity vR_b[3] "relative velocity in body frame";
    SI.Acceleration aR_b[3] "relative acceleration in body frame";
    
    SI.Velocity vtTol = 0.01 "velocity at which to ignore aerodynamics";
    SI.Acceleration vtDot "Derivative of true airspeed";
    SI.Angle alpha "angle of attack";
    SI.AngularVelocity alphaDot "angle of attack derivative";
    SI.Angle beta "side slip angle";
    SI.Angle betaDot "side slip angle derivative";
    SI.Pressure qBar "average dynamics pressure";
    Real g_n[3];
  end Base;

  model FlatEarth6DOF

    extends Base;
    Real eulerRates_wb[3,3];

  equation

    // the aircraft is the root of the 
    // local coordinate system
    frame.r_0 = {0,0,0};
    frame.C_0f = identity(3);

    // force and moment
    g_n = world.g_n(asl);
    F_b = -frame.f_0 + C_bn*g_n;
    M_b = -frame.t_0;

    // rotatoin matrices
    C_bn = Frames.T1(phi)*Frames.T2(theta)*Frames.T3(psi);
    C_nb = transpose(C_bn);
    eulerRates_wb  = 
      {{1, tan(theta)*sin(phi), tan(theta)*cos(phi)},
       {0,            cos(phi),           -sin(phi)},
       {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};

    // dynamics
    H = J*w_b;
    der(H) + cross(w_b,H) = M_b;  
    v_n = C_nb*v_b;
    der(v_b) = a_b;
    a_b + cross(w_b,v_b) = F_b/m;
    der({phi,theta,psi}) = eulerRates_wb*w_b;

    // navigation equations
    der(lat) = v_n[1]/(world.M(lat)+asl);
    der(lng) = v_n[2]/(world.N(lat)+asl)/cos(lat);
    der(asl) = -v_n[3];

    // aerodynamics
    vR_b = C_bn*(v_n - world.wind_n(lat,lng,asl));
    aR_b = der(vR_b);
    qBar = 0.5*world.rho(asl)*vt^2;

    // avoid singularity in side slip angle/ vt
    // when magnitude of vt is negligible
    if (sqrt(vR_b*vR_b) > vtTol) then
      vt = sqrt(vR_b*vR_b);
      beta = asin(vR_b[2]/vt);
      // omc doesn't like der(beta), setting manually
      betaDot = (aR_b[2]*vt - aR_b[2]*vtDot) /
        vt*sqrt(vR_b[1]^2 + vR_b[3]^2);
      // omc doesn't like der(vt), setting manually
      vtDot = (vR_b[1]*aR_b[1] + 
            vR_b[2]*aR_b[2] +
            vR_b[3]*aR_b[3])/vt;
    else
      vt = 0;
      beta = 0;
      betaDot = 0;
      vtDot = 0;
    end if;

    // avoid singularity in alpha when relative
    // forward velocity is zero
    if ( abs(vR_b[1]) > vtTol) then
      alpha = atan2(vR_b[3],vR_b[1]);
      // omc doesn't like der(alpha), setting manually
      alphaDot = (vR_b[1]*aR_b[3]-vR_b[3]*aR_b[1])/
        (vR_b[1]^2 + vR_b[3]^2); //stevens & lewis pg 78
    else
      alpha = 0;
      alphaDot = 0;
    end if;

  end FlatEarth6DOF;

end Aircraft;

// vim:ts=2:sw=2:expandtab:

