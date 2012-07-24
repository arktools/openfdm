within OpenFDM;

package SIunits

  package Conversions

    import SI = Modelica.SIunits;

    type Length_ft = Real(
      final quantity="Length",
      final unit="ft") "Length in feet";

    function to_ft
      input SI.Length meters;
      output Length_ft feet;
    algorithm
      feet := meters * 3.281;
    end to_ft;

    function from_ft
      input Length_ft feet;
      output SI.Length meters;
    algorithm
      meters := 0.3048*feet;
    end from_ft;

    type AngularVelocity_degs = Real (
      final quantity="AngularVelocity",
      final unit="deg/s")
      "Angular velocity in degrees per second";

    function to_degs
      input SI.AngularVelocity rads;
      output AngularVelocity_degs degs;
    algorithm
      degs := 57.2957795*rads;
    end to_degs;

    function from_degs
      input AngularVelocity_degs degs;
      output SI.AngularVelocity rads;
    algorithm
      rads := 0.0174532925*degs;
    end from_degs;

  end Conversions;
  
end SIunits;
