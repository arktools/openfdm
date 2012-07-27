package Trim

model Ex1
  import OpenFDM.Aerodynamics.Examples.*;
  SimpleForceAndTorqueEx example1;
    Real a;
    Real b;
    Real c;
equation
    der(a) = a*c*3-a*c*b;
    der(b) = b+c;
    der(c) = c+3;
initial equation
    der(a) = 0;
    der(b) = 0;
    der(c) = 0;
end Ex1;

end Trim;
