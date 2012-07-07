package OpenFDM
  package Kinematics
    model FlatEarthBodyAxes6DOF
      import Modelica.SIunits.*;
      // parameters
      parameter MomentOfInertia Jx "moment of inertia about x";
      parameter MomentOfInertia Jy "moment of inertia about y";
      parameter MomentOfInertia Jz "moment of inertia about z";
      parameter MomentOfInertia Jxz "cross moment of inertia about x/z";
      parameter Acceleration gD "gravitational acceleration in down direction";
      parameter Mass m "mass of vehicle";
      // state
      Velocity u "forward velocity in body frame";
      Velocity v "side velocity in body frame";
      Velocity w "down velocity in body frame";
      AngularVelocity p "body roll rate";
      AngularVelocity q "body pitch rate";
      AngularVelocity r "body yaw rate";
      Angle phi "roll angle";
      Angle theta "angle above horizon";
      Angle psi "heading";
      Distance pN "distance in north direction";
      Distance pE "distance in east direction";
      Distance h "altitude";
      // temporary variables
      Real gamma = Jx * Jy + Jz ^ 2;
      // connection
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a XYZ annotation(Placement(visible = true, transformation(origin = {-119.332,8.75051}, extent = {{-12,-12},{12,12}}, rotation = 0), iconTransformation(origin = {-119.332,8.75051}, extent = {{-12,-12},{12,12}}, rotation = 0)));
      // icon
      annotation(Documentation(info = "<html>
see Aircraft Control and Simulation, Lewis and Stevens pg. 110
</html>"), Diagram(), Icon(graphics = {Rectangle(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{-92.3077,90.9091},{91.6084,-90.9091}}),Text(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{111.466,-10.8669},{143.331,-34.4914}}, textString = "ECEF"),Text(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{-135.095,-8.75051},{-108.521,-29.7295}}, textString = "XYZ"),Text(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{-85.2407,74.2036},{96.5775,42.0358}}, textString = "Flat Earth"),Text(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{-90.0026,18.1189},{91.8156,-14.0489}}, textString = "Body Axes"),Text(rotation = 0, lineColor = {0,0,255}, fillColor = {0,0,0}, pattern = LinePattern.Solid, fillPattern = FillPattern.None, lineThickness = 0.25, extent = {{-89.4735,-37.9657},{92.3447,-70.1335}}, textString = "6DOF")}));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a XYZ annotation(Placement(visible = true, transformation(origin = {-122.507,6.10501}, extent = {{-12,-12},{12,12}}, rotation = 0), iconTransformation(origin = {-122.507,6.10501}, extent = {{-12,-12},{12,12}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b ECEF annotation(Placement(visible = true, transformation(origin = {126.455,1.05821}, extent = {{-12,-12},{12,12}}, rotation = 0), iconTransformation(origin = {126.455,1.05821}, extent = {{-12,-12},{12,12}}, rotation = 0)));
    equation
      der(u) = r * v - q * w - gD * sin(theta) + XYZ.f[1] / m;
      der(v) = -r * u - q * w - gD * sin(phi) * cos(theta) + XYZ.f[2] / m;
      der(w) = q * u - p * v + gD * cos(phi) * cos(theta) + XYZ.f[3] / m;
      der(phi) = p + tan(theta) * (q * sin(phi) + r * cos(phi));
      der(theta) = q * cos(phi) - r * sin(phi);
      der(psi) = (q * sin(phi) + r * cos(phi)) / cos(theta);
      gamma * der(p) = Jxz * (Jx - Jy + Jz) * p * q - (Jz * (Jz - Jy) + Jxz ^ 2) * q * r + Jz * XYZ.t[1] + Jxz * XYZ.t[3];
      Jy * der(q) = (Jz - Jx) * p * r - Jxz * (p ^ 2 - r ^ 2) + m;
      gamma * der(r) = ((Jx - Jy) * Jx + Jxz ^ 2) * p * q - Jxz * (Jx - Jy + Jz) * q * r + Jxz * XYZ.t[1] + Jx * XYZ.t[3];
      der(pN) = u * cos(theta) * cos(psi) + v * (-cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)) + w * (sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi));
      der(pE) = u * cos(theta) * sin(psi) + v * (cos(phi) * cos(psi) + sin(phi) * cos(theta) * sin(psi)) + w * (-sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi));
      der(h) = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta);
    end FlatEarthBodyAxes6DOF;
    // force equations
    // kinematic equations
    // moment equations
    // navigation equations      
    // connection equations
  end Kinematics;
  package Examples
    model Test
      OpenFDM.Kinematics.FlatEarthBodyAxes6DOF flatearthbodyaxes6dof1(Jx = 1, Jy = 1, Jz = 1, Jxz = 1, gD = 1, m = 1) annotation(Placement(visible = true, transformation(origin = {19.9666,-18.657}, extent = {{-12,-12},{12,12}}, rotation = 0)));
    end Test;
  end Examples;
end OpenFDM;

