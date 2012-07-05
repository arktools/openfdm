package Aircraft
  package Aerodynamics
  end Aerodynamics;
  package Atmosphere
  end Atmosphere;
  package Bodies
    model Body
      annotation(Diagram(), Icon());
      Aircraft.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {0.00874534,105.472}, extent = {{-12,-12},{12,12}}, rotation = 0), iconTransformation(origin = {0.00874534,105.472}, extent = {{-12,12},{12,-12}}, rotation = -90)));
    end Body;
  end Bodies;
  package Examples
    model Test
      Aircraft.Bodies.Body body1 annotation(Placement(visible = true, transformation(origin = {-40.9938,44.9689}, extent = {{-12,-12},{12,12}}, rotation = 0)));
    end Test;
  end Examples;
  package Functions
  end Functions;
  package Icons
  end Icons;
  package Interfaces
    connector Frame "Connector for aerodynamic and mechanical purposes"
      SI.Pressure p "Static pressure";
      SI.Density rho "Atmosphere density";
      SI.Temperature T "Static temperature";
      Types.TransferMatrix Tmat "Transfer matrix from local to inertial system";
      SI.Position r[3] "Position of frame relative inertial frame";
      SI.AngularVelocity w[3] "Angular velocity";
      SI.Velocity v[3] "Translational velocity in earth frame";
      SI.Acceleration g[3] "Gravitational acceleration";
      flow SI.Force[3] F "Force";
      flow SI.Torque[3] M "Torque";
      flow SI.Mass m "Mass";
      flow Real[3] md(unit = "kg.m") "Product mass*position (m*d) for calc CoG";
      flow SI.MomentOfInertia[3,3] I "Mass moment of inertia";
    end Frame;
    connector Frame_a "Coordinate system fixed to the component with one cut-force and cut-torque (filled rectangular icon)"
      extends Frame;
      annotation(defaultComponentName = "frame_a", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}, grid = {1,1}, initialScale = 0.16), graphics = {Rectangle(extent = {{-10,10},{10,-10}}, lineColor = {95,95,95}, lineThickness = 0.5),Rectangle(extent = {{-30,100},{30,-100}}, lineColor = {0,0,0}, fillColor = {192,192,192}, fillPattern = FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}, grid = {1,1}, initialScale = 0.16), graphics = {Text(extent = {{-140,-50},{140,-88}}, lineColor = {0,0,0}, textString = "%name"),Rectangle(extent = {{-12,40},{12,-40}}, lineColor = {0,0,0}, fillColor = {192,192,192}, fillPattern = FillPattern.Solid)}), Documentation(info = "<html>
<p>
Basic definition of a coordinate system that is fixed to a mechanical
component. In the origin of the coordinate system the cut-force
and the cut-torque is acting.
This component has a filled rectangular icon.
</p>
</html>"));
    end Frame_a;
    connector Frame_b "Coordinate system fixed to the component with one cut-force and cut-torque (non-filled rectangular icon)"
      extends Frame;
      annotation(defaultComponentName = "frame_b", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}, grid = {1,1}, initialScale = 0.16), graphics = {Rectangle(extent = {{-10,10},{10,-10}}, lineColor = {95,95,95}, lineThickness = 0.5),Rectangle(extent = {{-30,100},{30,-100}}, lineColor = {0,0,0}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}, grid = {1,1}, initialScale = 0.16), graphics = {Text(extent = {{-140,-50},{140,-88}}, lineColor = {0,0,0}, textString = "%name"),Rectangle(extent = {{-12,40},{12,-40}}, lineColor = {0,0,0}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid)}), Documentation(info = "<html>
<p>
Basic definition of a coordinate system that is fixed to a mechanical
component. In the origin of the coordinate system the cut-force
and the cut-torque is acting. This component has a non-filled rectangular icon.
</p>
</html>"));
    end Frame_b;
  end Interfaces;
  package Propulsion
  end Propulsion;
  package Test
  end Test;
  package Transformations
  end Transformations;
  package Types
    type TransferMatrix = Real[3,3] "A standard 3x3 rotation matrix";
    type Quaternion = Real[4] "Quaternion {q0,q1,q2,q3}";
    type EulerAngle = Modelica.SI.Angle[3] "Euler angles {psi,theta,phi}";
  end Types;
  package Utilities
  end Utilities;
end Aircraft;

