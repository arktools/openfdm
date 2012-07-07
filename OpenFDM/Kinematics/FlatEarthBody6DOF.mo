within OpenFDM.Kinematics;

model FlatEarthBody6DOF "Flat earth, body axes, 6 degree of freedom, equations of motion"
  
  // see Aircraft Control/ Simulation Lewis/ Stevens pg. 110
  
  import SI=Modelica.SIunits;

  // parameters
  
  parameter SI.Mass m;
  parameter SI.MomentOfInertia Jx;
  parameter SI.MomentOfInertia Jy;
  parameter SI.MomentOfInertia Jz;
  parameter SI.MomentOfInertia Jxz;
  parameter SI.Acceleration gD = 9.8;
  
  // state
  
  SI.Velocity u(start=0) "forward velocity";
  SI.Velocity v(start=0) "side velocity";
  SI.Velocity w(start=0) "down velocity";
   
  SI.Angle phi(start=0) "roll, euler angle 1";
  SI.Angle theta(start=0) "pitch, euler angle 2";
  SI.Angle psi(start=0) "heading, euler angle 3";
  
  SI.AngularVelocity p(start=0) "roll rate";
  SI.AngularVelocity q(start=0) "body pitch rate";
  SI.AngularVelocity r(start=0) "body yaw rate";
  
  SI.Distance pN(start=0) "north position";
  SI.Distance pE(start=0) "east position";
  SI.Distance h(start=0) "altitude";
  
  // connections
  //Modelica.Mechanics.MultiBody.Interfaces.Frame_a XYZ;

  // convenience variables
  Real gamma = Jx * Jy - Jxz^2;
  
equation
  
  // force equations
  der(u) = r * v - q * w - gD * sin(theta); // + XYZ.f[1]/m;
  der(v) = - r * u + p * w + gD * sin(phi) * cos(theta); // + XYZ.f[2]/m;
  der(w) = q * u - p * v + gD * cos(phi) * cos(theta); // + XYZ.f[3]/m;
  
  // attitude kinematics
  der(phi) = p + tan(theta) * (q * sin(phi) + r * cos(phi));
  der(theta) = q * cos(phi) - r * sin(phi);
  der(psi) = (q * sin(phi) + r * cos(phi)) / cos(theta);
  
  // moment equations
  gamma * der(p) = Jxz * (Jx - Jy + Jz) * p * q - (Jz * (Jz - Jy) + Jxz^2) * q * r; // + Jz * XYZ.t[1] + Jxz * XYZ.t[3];
  Jy * der(q) = (Jz - Jx) * p * r - Jxz * (p^2 - r^2); // + XYZ.t[2];
  gamma * der(r) = ((Jx - Jy) * Jx + Jxz^2) * p * q - Jz * (Jx - Jy + Jz) * q * r; // + Jxz * XYZ.t[1] + Jx * XYZ.t[3];
  
  // navigation equations
  der(pN) = u * cos(theta) * cos(psi) + v * (-cos(phi) * cos(psi)) + w * (sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi));
  der(pE) = u * cos(theta) * sin(psi) + v * cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi) + w * (-sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi));
  der(h) = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta);

end FlatEarthBody6DOF;