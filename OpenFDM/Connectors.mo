within OpenFDM;

package Connectors

  import SI=Modelica.SIunits;

  expandable connector Frame
  " based on multibody
    assumed force/torque in body 
    frame for simplicity,

    ***Note**
    this is different MultiBody in that the
    center of the coordinate system is the rigid body"

    // translation
    flow SI.Force f_0[3];
    SI.Position r_0[3]; // w.r.t. cm

    // rotation
    flow SI.Torque t_0[3];
    Real C_0f[3,3]; // rotation matrix from frame to body reference

  end Frame;

end Connectors;

// vim:ts=2:sw=2:expandtab
