package OpenFDM
  package Kinematics
    model FlatEarthBodyAxes6DOF
      import Modelica.SIunits.*;
      Velocity u "forward velocity in body frame";
      Velocity v "side velocity in body frame";
      Velocity w "down velocity in body frame";
      AngularVelocity p "roll rate";
      AngularVelocity q "pitch rate";
      annotation(Diagram(), Icon());
    equation
      der(u) = r * v - q * w - gD * sin(theta);
      der(v) = 0;
      der(w) = 0;
    end FlatEarthBodyAxes6DOF;
  end Kinematics;
  package Examples
    model Test
      OpenFDM.Kinematics.FlatEarthBodyAxes6DOF flatearthbodyaxes6dof1 annotation(Placement(visible = true, transformation(origin = {-6.87831,25.3968}, extent = {{-12,-12},{12,12}}, rotation = 0)));
    end Test;
  end Examples;
end OpenFDM;

