within OpenFDM;

package World

  import SI=Modelica.SIunits;

  model Base

    parameter SI.Acceleration gD;
    parameter SI.Density rho0;
    parameter SI.Length a"semi-major axis";
    parameter Real f "(a-b)/a";

    Real e = sqrt(a^2-b^2)/a "eccentricity";
    Real e2 = e^2;
    SI.Length b = a*(1-f) "semi-minor axis"; 

    function M = MBase(a=a,e=e);
    function N = NBase(a=a,e=e);
    function rho = rhoBase(rho0=rho0);
    function g_n = g_nBase(gD=gD);
    function wind_n = wind_nBase();
    function ground = groundBase();

  protected

    function MBase "meridian radius of curvature"
      input SI.Angle lat;
      input SI.Length a;
      input Real e;
      output SI.Length M;
    algorithm  
      M := a*(1-e^2)/(1-e^2*sin(lat)^2)^(3/2);
    annotation(Inline=true);
    end MBase;

    function NBase "prime vertical radius of curvature"
      input SI.Angle lat;
      input SI.Length a;
      input Real e;
      output SI.Length N;
    algorithm  
      N := a/sqrt(1-e^2*sin(lat)^2);
    annotation(Inline=true);
    end NBase;

    function rhoBase "air density"
      input SI.Position asl;
      input SI.Density rho0;
      output SI.Density rho;
    algorithm
      rho := rho0;
    annotation(Inline=true);
    end rhoBase;

    function g_nBase "gravity vector in nav frame"
      input SI.Position asl;
      input SI.Acceleration gD;
      output SI.Acceleration[3] g_n;
    algorithm
      g_n := {0,0,gD};
    annotation(Inline=true);
    end g_nBase;

    function wind_nBase "wind vector in nav frame"
      input SI.Angle lat;
      input SI.Angle lng;
      input SI.Position asl;
      output SI.Velocity[3] wind_n;
    algorithm
      wind_n := {0,0,0};
    annotation(Inline=true);
    end wind_nBase;

    function groundBase "wind vector in nav frame"
      input SI.Angle lat;
      input SI.Angle lng;
      output SI.Position agl;
    algorithm
      agl := 0;
    annotation(Inline=true);
    end groundBase;

  end Base;

  model Earth
    extends Base(
      gD = 9.80665,
      rho0 = 1.225,
      a = 6378137.0,
      f =  1/298.257223563);
  end Earth;

end World;

// vim:ts=2:sw=2:expandtab:
