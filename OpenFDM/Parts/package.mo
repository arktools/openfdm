within  OpenFDM;

package Parts

function T1
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{  1,      0,      0},
        {  0, cos(a), sin(a)},
        {  0,-sin(a), cos(a)}};
annotation(Inline=true);
end T1;

function T2
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{ cos(a),  0,-sin(a)},
        {  0,      1,      0},
        { sin(a),  0, cos(a)}};
annotation(Inline=true);
end T2;

function T3
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{ cos(a), sin(a), 0},
        {-sin(a), cos(a), 0},
        {      0,      0, 1}};
annotation(Inline=true);
end T3;

model ForceMoment "A rigid body force and moment."
  import SI=Modelica.SIunits;
  SI.Force F_b[3];
  SI.Torque M_b[3];
  Interfaces.RigidConnector fA;
equation
  fA.F_b + F_b = zeros(3);
  fA.M_b + M_b = zeros(3);
end ForceMoment;

partial model Translational "Translational kinematics of a rigid body. Requires moment definition to be complete. Used as a base for point mass and rigid body."
  outer World.WorldBase world;
  import SI=Modelica.SIunits;
  Interfaces.RigidConnector fA;
  SI.Mass m "mass";
  SI.Momentum L_b[3] "linear momentum";
equation
  L_b = m*(fA.v_b /*+ cross(fA.w_ib,fA.C_br*fA.r_r)*/);
  fA.F_b + fA.C_br*world.g_r(fA.r_r) = der(L_b) /*+ cross(fA.w_ib, L_b)*/;
end Translational;

model PointMass "A point mass with translational, but no rotational dynamics."
  extends Translational;
equation
  fA.M_b = zeros(3);
end PointMass;

model RigidBody "A body with rotational and translational dynamics. The body is assumed to be rigid."
  import SI=Modelica.SIunits;
  extends Translational;
  SI.MomentOfInertia I_b[3,3] "inertial about the cm in the body frame";
protected
  SI.AngularMomentum H_b[3] "angular momentum";
equation
  H_b = I_b*fA.w_ib;
  fA.M_b = der(H_b) + cross(fA.w_ib,H_b);
end RigidBody;

partial model RigidLink "Requires C_ba definition to be complete."
  import SI=Modelica.SIunits;
  input SI.Position r_a[3] "position vector from fA to fB, resolved in fA";
  input SI.Angle angles[3] "rotation angles from fA into fB";
  Interfaces.RigidConnector fA, fB;
  Real C_ba[3,3] "rotation matrix from fA into fB";
equation
  fA.r_r + transpose(fA.C_br)*r_a = fB.r_r;
  C_ba*fA.v_b = fB.v_b;
  C_ba*fA.a_b = fB.a_b;
  zeros(3) = C_ba*fA.F_b + fB.F_b;
  C_ba*fA.C_br = fB.C_br;
  C_ba*fA.w_ib = fB.w_ib;
  C_ba*fA.z_b = fB.z_b;
  zeros(3) = C_ba*fA.M_b + 
    cross(C_ba*r_a,fB.F_b) +
    fB.M_b;
end RigidLink;

model RigidLink_B321 "A body 3-2-1 rotation sequence rigid connector"
  extends RigidLink;
equation
  C_ba = T1(angles[1])*T2(angles[2])*T3(angles[3]);
end RigidLink_B321;

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
  SI.Angle euler[3](each stateSelect=StateSelect.always) "euler angles, body roll, horizon pitch, heading";
  SI.AngularVelocity w_ib[3](each stateSelect=StateSelect.always) "angular velocity of body wrt inertial frame resolved in the body frame";

  // auxiliary variables
  SI.Angle eulerDot[3](each stateSelect=StateSelect.never) "euler angle rates, body roll, horizon pitch, heading";
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
  SI.Angle phiDot = eulerDot[1] "euler angle 1 rate: body roll";
  SI.Angle thetaDot = eulerDot[2] "euler angle 2 rate: horizon pitch";
  SI.Angle psiDot = eulerDot[3] "euler angle 3 rate: heading";

  Real phi_deg = SI.Conversions.to_deg(phi);
  Real theta_deg = SI.Conversions.to_deg(theta);
  Real psi_deg = SI.Conversions.to_deg(psi);
  constant Real epsilon = 1e-14;
  Real C_euler[3,3];

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
  eulerDot = der(euler);
  C_euler = {
     {1,           tan(theta)*sin(phi),           tan(theta)*cos(phi)},
     {0,                      cos(phi),                     -sin(phi)},
     {0, sin(phi)/(cos(theta)+epsilon), cos(phi)/(cos(theta)+epsilon)}};
  w_ib = w_ir + C_euler * der(euler);
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

end Parts;

// vim:ts=2:sw=2:expandtab:
