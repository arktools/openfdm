within OpenFDM;

model RigidReferencePoint "The reference point of a rigid body. The acceleratoin and velocity are calculated here and passed through the rigid connector to the rest of the rigid body components. Convenience variables (e.g. roll/pitch/heading) are also defined as this is the point of interest for the rigid body."

  import SI=Modelica.SIunits;
  import C=Modelica.Constants;

  outer World.WorldBase world "the world";
  Interfaces.RigidConnector fA "the rigid body connector";
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
  SI.Position agl(stateSelect=StateSelect.never) "altitude above ground level";

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

  // assertion
  agl = world.agl(r_r);
  assert(agl > 0, "hit ground");

end RigidReferencePoint;

// vim:ts=2:sw=2:expandtab
