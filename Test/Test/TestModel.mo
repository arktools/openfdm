within Test;

model TestModel
  
  Real a(start=0);
  Real b(start=0);

equation
  
  der(a) = b;
  der(b) = 1;

end TestModel;