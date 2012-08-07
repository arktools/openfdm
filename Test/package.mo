package Test

  import C=Modelica.Constants;
  import SI=Modelica.SIunits;

  function T1
    input SI.Angle a;
    output Real T[3,3];
  algorithm
    T := {{  1,      0,      0},
          {  0, cos(a), sin(a)},
          {  0,-sin(a), cos(a)}};
  annotation(Inline=true);
  end T1;

  function T2
    input SI.Angle a;
    output Real T[3,3];
  algorithm
    T := {{ cos(a),  0,-sin(a)},
          {  0,      1,      0},
          { sin(a),  0, cos(a)}};
  annotation(Inline=true);
  end T2;

  function T3
    input SI.Angle a;
    output Real T[3,3];
  algorithm
    T := {{ cos(a), sin(a), 0},
          {-sin(a), cos(a), 0},
          {      0,      0, 1}};
  annotation(Inline=true);
  end T3;

  model World

    function g_r
      input SI.Position r_r[3];
      output SI.Acceleration g_r[3];
    algorithm
      g_r := {0,0,9.8};
    annotation(Inline=true);
    end g_r;

    function agl
      input SI.Position r_r[3];
      output SI.Position agl;
    algorithm
      agl := -r_r[3];
    annotation(Inline=true);
    end agl;

    function rho
      input SI.Position r_r[3];
      output SI.Density rho;
    algorithm
      rho := 1.225;
    annotation(Inline=true);
    end rho;

    function wind_r
      input SI.Position r_r[3];
      output SI.Velocity wind_r[3];
    algorithm
      wind_r := {0,0,0};
    annotation(Inline=true);
    end wind_r;

  end World;

  expandable connector RigidConnector "A connector for rigid body components. Expandable to avoid potential/ flow balance warning. The rigid connection has more potential variable due to the rigid connection passing velocity, acceleration information that would be redundant to calculate"
    SI.Position r_r[3];
    SI.Velocity v_b[3];
    SI.Acceleration a_b[3];
    flow SI.Force F_b[3];
    Real C_br[3,3];
    SI.AngularVelocity w_ib[3];
    SI.AngularAcceleration z_b[3]; 
    flow SI.Torque M_b[3];
  end RigidConnector;

  partial model RigidLink "Requires C_ba definition to be complete."
    input SI.Position r_a[3];
    input SI.Angle angles[3];
    RigidConnector fA, fB;
    Real C_ba[3,3];
  equation
    fA.r_r + transpose(fA.C_br)*r_a = fB.r_r;
    C_ba*fA.v_b = fB.v_b;
    C_ba*fA.a_b = fB.a_b;
    C_ba*fA.F_b + fB.F_b = zeros(3);
    C_ba*fA.C_br = fB.C_br;
    C_ba*fA.w_ib = fB.w_ib;
    C_ba*fA.z_b = fB.z_b;
    C_ba*fA.M_b + fB.M_b = zeros(3);
  end RigidLink;

  model RigidLink_B321 "A body 3-2-1 rotation sequence rigid connector"
    extends RigidLink;
  equation
    C_ba = T1(angles[1])*T2(angles[2])*T3(angles[3]);
  end RigidLink_B321;

  model ForceMoment "A rigid body force and moment."
    SI.Force F_b[3];
    SI.Torque M_b[3];
    RigidConnector fA;
  equation
    fA.F_b + F_b = zeros(3);
    fA.M_b + M_b = zeros(3);
  end ForceMoment;

  partial model TranslationalDynamics "Translational dynamics of a rigid body. Requires moment definition to be complete. Used as a base for point mass and rigid body."
    outer World world;
    RigidConnector fA;
    SI.Mass m "mass";
  protected
    SI.Momentum L_b[3] "linear momentum";
  equation
    L_b = m*(fA.v_b + cross(fA.w_ib,fA.C_br*fA.r_r));
    fA.F_b + fA.C_br*world.g_r(fA.r_r) = 
      der(L_b) + cross(fA.w_ib,L_b);
  end TranslationalDynamics;

  model PointMass "A point mass with translationa, but rotational dynamics."
    extends TranslationalDynamics;
  equation
    fA.M_b = zeros(3);
  end PointMass;

  model RigidBody "A body with rotational and translational dynamics. The body is assumed to be rigid."
    extends TranslationalDynamics;
    SI.MomentOfInertia I_b[3,3] "inertial about the cm in the body frame";
  protected
    SI.AngularMomentum H_b[3] "angular momentum";
  equation
    H_b = I_b*fA.w_ib;
    fA.M_b = der(H_b) + cross(fA.w_ib,H_b);
  end RigidBody;

  model RigidReferencePoint "The reference point of a rigid body. The acceleratoin and velocity are calculated here and passed through the rigid connector to the rest of the rigid body components. Convenience variables (e.g. roll/pitch/heading) are also defined as this is the point of interest for the rigid body."
    RigidConnector fA "the rigid body connector";
    SI.AngularVelocity w_ir[3] = {0,0,0} "ref frame ang rate wrt inertial expressed in the reference frame";
    
    // states
    SI.Position r_r[3](each stateSelect=StateSelect.always) "cartesian position resolved in the refernce frame";
    SI.Velocity v_b[3](each stateSelect=StateSelect.always) "velocity resolved in the body frame";
    SI.Angle euler[3](each stateSelect=StateSelect.always)
      "euler angles, body roll, horizon pitch, heading";
    SI.AngularVelocity w_ib[3](each stateSelect=StateSelect.always) "angular velocity of body wrt inertial frame resolved in the body frame";

    SI.Acceleration a_b[3](each stateSelect=StateSelect.never) "acceleration resolved in the body frame";
    SI.AngularAcceleration z_b[3](each stateSelect=StateSelect.never) "angular acceleration resolved in the body frame";
    Real C_br[3,3](each stateSelect=StateSelect.never) "direction cosine matrix  from reference to body frame";

    // alias's
    SI.Angle roll = euler[1] "euler angle 1: body roll";
    SI.Angle pitch = euler[2] "euler angle 2: horizon pitch";
    SI.Angle heading = euler[3] "euler angle 3: heading";

  equation
    // connect frame
    fA.r_r = r_r;
    fA.v_b = v_b;
    fA.a_b = a_b;
    fA.F_b = zeros(3);
    fA.C_br = C_br;
    fA.w_ib = w_ib; 
    fA.z_b = z_b;
    fA.M_b = zeros(3);

    // kinematics
    v_b = C_br*der(r_r);
    a_b = der(v_b);
    fA.w_ib = w_ir + der(euler); // TODO*/
    C_br = T1(euler[1])*T2(euler[2])*T3(euler[3]);
    z_b = der(w_ib);

    // angle wrap
    for i in 1:size(euler,1) loop
      when (euler[i] > C.pi) then
        reinit(euler[i],pre(euler[i])-2*C.pi);
      end when;
    end for;

  end RigidReferencePoint;

end Test;

// vim:ts=2:sw=2:expandtab: