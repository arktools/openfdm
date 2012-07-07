within OpenFDM.Examples;

model TestModel
  
  Kinematics.FlatEarthBody6DOF body(m=1,Jx=1,Jy=1,Jz=1,Jxz=0);
  Modelica.Mechanics.MultiBody.World world;
  
equation
  
  //connect(world.frame_b,body.XYZ);
  
end TestModel;