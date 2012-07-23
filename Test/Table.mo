within Test;

model TableTest
    import Modelica.Blocks.Tables.CombiTable2D;
    import Modelica.Blocks.Tables.CombiTable1D;
    CombiTable1D czTable(columns={2}, smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative, table = transpose(
   {{   -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45}, 
    { 0.770,  0.241, -0.100, -0.416, -0.731, -1.053, -1.366, -1.646, -1.917, -2.120, -2.248, -2.229}}))
    "lift coefficient table(aoa[deg])";
 
    CombiTable2D test1 (table= {{0,1,2},
                         {1,1,2},
                         {2,2,3}});
    Modelica.Blocks.Sources.Constant const[2](k={-7,2});
    output Real v, cz;
equation
    connect(const[1].y,test1.u1);
    connect(const[2].y,test1.u2);
    connect(const[1].y,czTable.u[1]);
    v = test1.y;
    cz = czTable.y[1];
end TableTest;
