within OpenFDM;

package Interfaces

expandable connector RigidConnector "A connector for rigid body components. Expandable to avoid potential/ flow balance warning. The rigid connection has more potential variable due to the rigid connection passing velocity, acceleration information that would be redundant to calculate"
  import SI=Modelica.SIunits;
  SI.Position r_r[3];
  SI.Velocity v_b[3];
  SI.Acceleration a_b[3];
  flow SI.Force F_b[3];
  Real C_br[3,3];
  SI.AngularVelocity w_ib[3];
  SI.AngularAcceleration z_b[3]; 
  flow SI.Torque M_b[3];
end RigidConnector;

end Interfaces;

// vim:ts=2:sw=2:expandtab:
