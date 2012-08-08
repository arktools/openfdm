within OpenFDM;

model RigidReferencePoint "The reference point of a rigid body. The acceleratoin and velocity are calculated here and passed through the rigid connector to the rest of the rigid body components. Convenience variables (e.g. roll/pitch/heading) are also defined as this is the point of interest for the rigid body."

  import SI=Modelica.SIunits;
  import C=Modelica.Constants;

  outer World.WorldBase world "the world";
  parameter Boolean wrapEuler = true;
  Interfaces.RigidConnector fA "the rigid body connector";
  SI.AngularVelocity w_ir[3] = {0,0,0} "ref frame ang rate wrt inertial expressed in the reference frame";
  
  // states
  SI.Position r_r[3](each stateSelect=StateSelect.always) "cartesian position resolved in the refernce frame";
  SI.Velocity v_r[3](each stateSelect=StateSelect.always) "velocity resolved in the reference frame";
  SI.Angle euler[3](each stateSelect=StateSelect.always)
    "euler angles, body roll, horizon pitch, heading";
  SI.AngularVelocity w_ib[3](each stateSelect=StateSelect.always) "angular velocity of body wrt inertial frame resolved in the body frame";

  SI.Velocity v_b[3](each stateSelect=StateSelect.never) "velocity resolved in the body frame";
  SI.Acceleration a_b[3](each stateSelect=StateSelect.never) "acceleration resolved in the body frame";
  SI.AngularAcceleration z_b[3](each stateSelect=StateSelect.never) "angular acceleration resolved in the body frame";
  Real C_br[3,3](each stateSelect=StateSelect.never) "direction cosine matrix  from reference to body frame";
  SI.Position agl(stateSelect=StateSelect.never) "altitude above ground level";
  SI.Angle gamma "flight path angle";
  SI.Velocity vt "true velocity";
  SI.Velocity vR_r[3] "relative air velocity in reference frame";

  // alias's
  SI.Angle phi = euler[1] "euler angle 1: body roll";
  SI.Angle theta = euler[2] "euler angle 2: horizon pitch";
  SI.Angle psi = euler[3] "euler angle 3: heading";
  constant Real epsilon = 1e-14;

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
  v_r = der(r_r);
  v_b = C_br*v_r;
  a_b = der(v_b);
  w_ib = w_ir + 
    {{1,           tan(theta)*sin(phi),           tan(theta)*cos(phi)},
     {0,                      cos(phi),                     -sin(phi)},
     {0, sin(phi)/(cos(theta)+epsilon), cos(phi)/(cos(theta)+epsilon)}} * der(euler);
  C_br = T1(phi)*T2(theta)*T3(psi);
  z_b = der(w_ib);

  // angle wrap
  if wrapEuler then
    for i in 1:size(euler,1) loop
      when (euler[i] > C.pi) then
        reinit(euler[i],pre(euler[i])-2*C.pi);
      end when;
      when (euler[i] < -C.pi) then
        reinit(euler[i],pre(euler[i])+2*C.pi);
      end when;
    end for;
  end if;

  // extra
  vR_r = v_r - world.wind_r(r_r);
  vt = sqrt(vR_r*vR_r); 
  gamma = asin(v_r[3]/sqrt(v_r*v_r + epsilon));

  // assertion
  agl = world.agl(r_r);
  assert(agl > 0, "hit ground");

end RigidReferencePoint;

// vim:ts=2:sw=2:expandtab
