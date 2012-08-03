within OpenFDM.Aerodynamics.Examples;

block CombiTable1DSISO
    output Real y1; 
    input Real u1;
    extends Modelica.Blocks.Tables.CombiTable1Ds(columns={2});
equation
    y[1] = y1; 
    u = u1;
end CombiTable1DSISO;

record DatcomCoefficients
    Real CL_Basic;
end DatcomCoefficients;

record DatcomTables
    constant Real[:,:] CL_Basic;
end DatcomTables;

model ForceAndTorque
    Real CL;
    Real L;
    Real alpha;
equation
    L = CL;
end ForceAndTorque;

model DatcomForceAndTorque
    extends ForceAndTorque;
    extends DatcomCoefficients;
equation
    CL = CL_Basic;
end DatcomForceAndTorque;

model DatcomForceAndTorqueTable
    extends DatcomForceAndTorque;
    parameter DatcomTables data; 
    CombiTable1DSISO CL_Basic_Table(
        table=data.CL_Basic,
        u1=alpha,y1=CL_Basic);   
end DatcomForceAndTorqueTable; 

package AircraftName
constant DatcomTables datcomTables(CL_Basic = {{0,0},{1,0}});
end AircraftName;

model DatcomForceAndTorqueTableTest
    DatcomForceAndTorqueTable aerodynamics(alpha=0,data=AircraftName.datcomTables);
end DatcomForceAndTorqueTableTest;
