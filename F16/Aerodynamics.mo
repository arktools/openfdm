within F16;
  
model Aerodynamics
  import Modelica.Blocks.Tables.*;

  // inputs
  input Real alpha "aoa [rad]";
  input Real beta "side slipe angle [rad]";
  //input Real vt;
  constant Real vt = 1.0; //DELETE LATER
  input Real p;
  input Real q;
  input Real r;
  //input Real b;
  constant Real b = 1.0;
  input Real cbar;
  input Real xcg;
  input Real xcgr;
  input Real elevator;
  input Real aileron;
  input Real rudder; 

  output Real cx;
  output Real cy;
  output Real cz;
  output Real cl;
  output Real cm;
  output Real cn;
  
  constant Real cxq_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {-0.267, -0.110,  0.308,   1.34,   2.08,   2.91,   2.76,   2.05,   1.50,   1.49,   1.83,   1.21}})
    "drag coefficient due to pitch rate table(aoa[deg])";
    
  constant Real cyr_table[12,2] = 
  transpose({{     -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    { 0.882,  0.852,  0.876,  0.958,  0.962,  0.974,  0.819,  0.483,  0.590,  0.121, -0.493,  -1.04}})
    "side coefficient due to yaw rate table(aoa[deg])";
    
  constant Real cyp_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {-0.108, -0.108, -0.188,  0.110,  0.258,  0.226,  0.344,  0.362,  0.611,  0.529,  0.298,  -2.27}})
    "side coefficient due to roll rate table(aoa[deg])";

  constant Real czp_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {-8.80,  -25.8,  -28.9,  -31.4,  -31.2,  -30.7,  -27.7,  -28.2,  -29.0,  -29.8,  -38.3,  -35.3}})
    "(what does table do??) table(aoa[deg])";
    
  constant Real clr_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45}, 
    {-0.126, -0.026,  0.063,  0.113,  0.208,  0.230,  0.319,  0.437,  0.680,  0.100,  0.447, -0.330}})
    "roll momment coefficient due to yaw rate table(aoa[deg])";
    
  constant Real clp_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {-0.360, -0.359, -0.443, -0.420, -0.383, -0.375, -0.329, -0.294, -0.230, -0.210, -0.120, -0.100}})
    "roll moment coefficient due to roll rate table(aoa[deg])";
    
  constant Real cmq_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    { -7.21, -0.540,  -5.23,  -5.26,  -6.11,  -6.64,  -5.69,  -6.00,  -6.20,  -6.40,  -6.60,  -6.00}})
    "pitch moment coefficient due to roll rate table(aoa[deg])";
    
  constant Real cnr_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {-0.380, -0.363, -0.378, -0.386, -0.370, -0.453, -0.550, -0.582, -0.595, -0.637,  -1.02, -0.840}})
    "yaw moment coefficient due to yaw rate table(aoa[deg])";
    
  constant Real cnp_table[12,2] = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    { 0.061,  0.052,  0.052, -0.012, -0.013, -0.024,  0.050,  0.150,  0.130,  0.158,  0.240,  0.150}})
    "yaw moment coefficient due to roll rate table(aoa[deg])";
  
  constant Real cx_table[6,13]  =    
  {{      0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {   -24,   -0.099, -0.081, -0.081,  0.063, -0.025,  0.044,  0.097,  0.113,  0.145,  0.167,  0.174,  0.166},
    {   -12,   -0.048,  0.038, -0.040, -0.021,  0.016,  0.083,  0.127,  0.137,  0.162,  0.177,  0.179,  0.167},
    {     0,   -0.022, -0.020, -0.021, -0.004,  0.032,  0.094,  0.128,  0.130,  0.154,  0.161,  0.155,  0.138},
    {    12,   -0.040, -0.038, -0.039, -0.025,  0.006,  0.062,  0.087,  0.085,  0.100,  0.110,  0.104,  0.091},
    {    24,   -0.083, -0.073, -0.076, -0.072, -0.046,  0.012,  0.024,  0.025,  0.043,  0.053,  0.047,  0.040}}
    "drag coefficient table(aoa[deg],elev[deg])";
  
  constant Real cz_table[12,2]  = 
  transpose({{    -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45}, 
    { 0.770,  0.241, -0.100, -0.416, -0.731, -1.053, -1.366, -1.646, -1.917, -2.120, -2.248, -2.229}})
    "lift coefficient table(aoa[deg])";
  
  constant Real cm_table[6,13]  =    
  {{      0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45}, 
    {   -24,    0.205,  0.168,  0.186,  0.196,  0.213,  0.251,  0.245,  0.238,  0.252,  0.231,  0.198,  0.192},
    {   -12,    0.081,  0.077,  0.107,  0.110,  0.110,  0.141,  0.127,  0.119,  0.133,  0.108,  0.081,  0.093},
    {     0,   -0.046, -0.020, -0.009, -0.005, -0.006,  0.010,  0.006, -0.001,  0.014,  0.000, -0.013,  0.032},
    {    12,   -0.174, -0.145, -0.121, -0.127, -0.129, -0.102, -0.097, -0.113, -0.087, -0.084, -0.069, -0.006},
    {    24,   -0.259, -0.202, -0.184, -0.193, -0.199, -0.150, -0.160, -0.167, -0.104, -0.076, -0.041, -0.005}}
    "pitch moment coefficient table(aoa[deg],abs(beta)[deg])";
  
  constant Real cl_table[8,13]  =    
  {{      0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {     0,    0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
    {     5,   -0.001, -0.004, -0.008, -0.012, -0.016, -0.019, -0.020, -0.020, -0.015, -0.008, -0.013, -0.015},
    {    10,   -0.003, -0.009, -0.017, -0.024, -0.030, -0.034, -0.040, -0.037, -0.016, -0.002, -0.010, -0.019},
    {    15,   -0.001, -0.010, -0.020, -0.030, -0.039, -0.044, -0.050, -0.049, -0.023, -0.006, -0.014, -0.027},
    {    20,    0.000, -0.010, -0.022, -0.034, -0.047, -0.046, -0.059, -0.061, -0.033, -0.036, -0.035, -0.035},
    {    25,    0.007, -0.010, -0.023, -0.034, -0.049, -0.046, -0.068, -0.071, -0.060, -0.058, -0.062, -0.059},
    {    30,    0.009, -0.011, -0.023, -0.037, -0.050, -0.047, -0.074, -0.079, -0.091, -0.076, -0.077, -0.076}}
    "roll moment coefficient table(aoa[deg],abs(beta)[deg])";
  
  
  constant Real cn_table[8,13] =
  {{     0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {     0,    0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
    {     5,    0.018,  0.019,  0.018,  0.019,  0.019,  0.018,  0.013,  0.007,  0.004, -0.014, -0.017, -0.033},
    {    10,    0.038,  0.042,  0.042,  0.042,  0.043,  0.039,  0.030,  0.017,  0.004, -0.035, -0.047, -0.057},
    {    15,    0.056,  0.057,  0.059,  0.058,  0.058,  0.053,  0.032,  0.012,  0.002, -0.046, -0.071, -0.073},
    {    20,    0.064,  0.077,  0.076,  0.074,  0.073,  0.057,  0.029,  0.007,  0.012, -0.034, -0.065, -0.041},
    {    25,    0.074,  0.086,  0.093,  0.089,  0.080,  0.062,  0.049,  0.022,  0.028, -0.012, -0.002, -0.013},
    {    30,    0.079,  0.090,  0.106,  0.106,  0.096,  0.080,  0.068,  0.030,  0.064,  0.015,  0.011, -0.001}}
    "yaw moment coefficient table(aoa[deg],abs(beta)[deg])";
  
  
  constant Real dlda_table[8,13] =
  {{     0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {   -30,   -0.041, -0.052, -0.053, -0.056, -0.050, -0.056, -0.082, -0.059, -0.042, -0.038, -0.027, -0.017},
    {   -20,   -0.041, -0.053, -0.053, -0.053, -0.050, -0.051, -0.066, -0.043, -0.038, -0.027, -0.023, -0.016},
    {   -10,   -0.042, -0.053, -0.052, -0.051, -0.049, -0.049, -0.043, -0.035, -0.026, -0.016, -0.018, -0.014},
    {     0,   -0.040, -0.052, -0.051, -0.052, -0.048, -0.048, -0.042, -0.037, -0.031, -0.026, -0.017, -0.012},
    {    10,   -0.043, -0.049, -0.048, -0.049, -0.043, -0.042, -0.042, -0.036, -0.025, -0.021, -0.016, -0.011},
    {    20,   -0.044, -0.048, -0.048, -0.047, -0.042, -0.041, -0.020, -0.028, -0.013, -0.014, -0.011, -0.010},
    {    30,   -0.043, -0.049, -0.047, -0.045, -0.042, -0.037, -0.003, -0.013, -0.010, -0.003, -0.007, -0.008}}
    "change in pitch moment coefficient due to aileron table(aoa[deg],abs(beta)[deg])";
  
  constant Real dldr_table[8,13]  =  
  {{     0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {   -30,    0.005,  0.017,  0.014,  0.010, -0.005,  0.009,  0.019,  0.005, -0.000, -0.005, -0.011,  0.008},
    {   -20,    0.007,  0.016,  0.014,  0.014,  0.013,  0.009,  0.012,  0.005,  0.000,  0.004,  0.009,  0.007},
    {   -10,    0.013,  0.013,  0.011,  0.012,  0.011,  0.009,  0.008,  0.005, -0.002,  0.005,  0.003,  0.005},
    {     0,    0.018,  0.015,  0.015,  0.014,  0.014,  0.014,  0.014,  0.015,  0.013,  0.011,  0.006,  0.001},
    {    10,    0.015,  0.014,  0.013,  0.013,  0.012,  0.011,  0.011,  0.010,  0.008,  0.008,  0.007,  0.003},
    {    20,    0.021,  0.011,  0.010,  0.011,  0.010,  0.009,  0.008,  0.010,  0.006,  0.005,  0.000,  0.001},
    {    30,    0.023,  0.010,  0.011,  0.011,  0.011,  0.010,  0.008,  0.010,  0.006,  0.014,  0.020,  0.000}}
    "change in roll moment coefficient due to rudder table(aoa[deg],abs(beta)[deg])";
  
  constant Real dnda_table[8,13]  =  
  {{     0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {   -30,    0.001, -0.027, -0.017, -0.013, -0.012, -0.016,  0.001,  0.017,  0.011,  0.017,  0.008,  0.016},
    {   -20,    0.002, -0.014, -0.016, -0.016, -0.014, -0.019, -0.021,  0.002,  0.012,  0.015,  0.015,  0.011},
    {   -10,   -0.006, -0.008, -0.006, -0.006, -0.005, -0.008, -0.005,  0.007,  0.004,  0.007,  0.006,  0.006},
    {     0,   -0.011, -0.011, -0.010, -0.009, -0.008, -0.006,  0.000,  0.004,  0.007,  0.010,  0.004,  0.010},
    {    10,   -0.015, -0.015, -0.014, -0.012, -0.011, -0.008, -0.002,  0.002,  0.006,  0.012,  0.011,  0.011},
    {    20,   -0.024, -0.010, -0.004, -0.002, -0.001,  0.003,  0.014,  0.006, -0.001,  0.004,  0.004,  0.006},
    {    30,   -0.022,  0.002, -0.003, -0.005, -0.003, -0.001, -0.009, -0.009, -0.001,  0.003, -0.002,  0.001}}
    "change in yaw moment coefficient due to aileron table(aoa[deg],abs(beta)[deg])";
  
  constant Real dndr_table[8,13]  =  
  {{     0,      -10,     -5,      0,      5,     10,     15,     20,     25,     30,     35,     40,     45},
    {   -30,   -0.018, -0.052, -0.052, -0.052, -0.054, -0.049, -0.059, -0.051, -0.030, -0.037, -0.026, -0.013},
    {   -20,   -0.028, -0.051, -0.043, -0.046, -0.045, -0.049, -0.057, -0.052, -0.030, -0.033, -0.030, -0.008},
    {   -10,   -0.037, -0.041, -0.038, -0.040, -0.040, -0.038, -0.037, -0.030, -0.027, -0.024, -0.019, -0.013},
    {     0,   -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047, -0.048, -0.049, -0.045, -0.033, -0.016},
    {    10,   -0.043, -0.044, -0.041, -0.041, -0.040, -0.038, -0.034, -0.035, -0.035, -0.029, -0.022, -0.009},
    {    20,   -0.052, -0.034, -0.036, -0.036, -0.035, -0.028, -0.024, -0.023, -0.020, -0.016, -0.010, -0.014},
    {    30,   -0.062, -0.034, -0.027, -0.028, -0.027, -0.027, -0.023, -0.023, -0.019, -0.009, -0.025, -0.010}}
    "change in yaw moment coefficient due to rudder table(aoa[deg],abs(beta)[deg])";
  //tables
  CombiTable1D cxqTable(table=cxq_table, columns={2});
  CombiTable1D cyrTable(table=cyr_table, columns={2});
  CombiTable1D cypTable(table=cyp_table, columns={2});
  CombiTable1D czpTable(table=czp_table, columns={2});
  CombiTable1D clrTable(table=clr_table, columns={2});
  CombiTable1D clpTable(table=clp_table, columns={2});
  CombiTable1D cmqTable(table=cmq_table, columns={2});
  CombiTable1D cnrTable(table=cnr_table, columns={2});
  CombiTable1D cnpTable(table=cnp_table, columns={2});
  CombiTable1D czTable(table=cz_table, columns={2});
  CombiTable2D cxTable(table=cx_table);
  CombiTable2D cmTable(table=cm_table);
  CombiTable2D clTable(table=cl_table);
  CombiTable2D cnTable(table=cn_table);
  CombiTable2D dldaTable(table=dlda_table);
  CombiTable2D dldrTable(table=dldr_table);
  CombiTable2D dndaTable(table=dnda_table);
  CombiTable2D dndrTable(table=dndr_table);
  //table output
  Real cxq, cyr, cyp, czp, clr, clp, cmq, cnr, cnp;
  Real cx0, cy0, cz0, cl0, cm0, cn0, dlda, dldr, dnda, dndr;
protected
  // local var for table use
  constant Real rtod = 57.296;
  Real alpha_deg, beta_deg;
  Real tvt, b2v, cq, daileron, drudder; 
equation
  
  //Table connections: independent var
  connect(cxqTable.u[1], alpha_deg);
  connect(cyrTable.u[1], alpha_deg);
  connect(cypTable.u[1], alpha_deg);
  connect(czpTable.u[1], alpha_deg);
  connect(clrTable.u[1], alpha_deg);
  connect(clpTable.u[1], alpha_deg);
  connect(cmqTable.u[1], alpha_deg);
  connect(cnrTable.u[1], alpha_deg);
  connect(cnpTable.u[1], alpha_deg);
  connect(czTable.u[1], alpha_deg);
  connect(cxTable.u1, elevator);
  connect(cxTable.u2, alpha_deg);
  connect(cmTable.u1, beta_deg);
  connect(cmTable.u2, alpha_deg);
  connect(clTable.u1, beta_deg);
  connect(clTable.u2, alpha_deg);
  connect(cnTable.u1, beta_deg);
  connect(cnTable.u2, alpha_deg);
  connect(dldaTable.u1, beta_deg);
  connect(dldaTable.u2, alpha_deg);
  connect(dldrTable.u1, beta_deg);
  connect(dldrTable.u2, alpha_deg);
  connect(dndaTable.u1, beta_deg);
  connect(dndaTable.u2, alpha_deg);
  connect(dndrTable.u1, beta_deg);
  connect(dndrTable.u2, alpha_deg);
  
  //Table connections: dependent var
  connect(cxqTable.y[1], cxq);
  connect(cyrTable.y[1], cyr);
  connect(cypTable.y[1], cyp);
  connect(czpTable.y[1], czp);
  connect(clrTable.y[1], clr);
  connect(clpTable.y[1], clp);
  connect(cmqTable.y[1], cmq);
  connect(cnrTable.y[1], cnr);
  connect(cnpTable.y[1], cnp);
  connect(czTable.y[1], cz0);
  connect(cxTable.y, cx0);
  connect(cmTable.y, cm0);
  connect(clTable.y, cl0);
  connect(cnTable.y, cn0);
  connect(dldrTable.y, dldr);
  connect(dldaTable.y, dlda);
  connect(dndaTable.y, dnda);
  connect(dndrTable.y, dndr);

  //Calculate local variables
  alpha_deg = alpha*rtod;
  beta_deg = beta*rtod;
  tvt = 0.5/vt;
  b2v = b*tvt;
  cq = cbar*q*tvt;
  daileron = aileron/20.0;
  drudder = rudder/30.0;

        cx = cx0 + cq * cxq;
        cy0 = -0.02*beta_deg + 0.021*(aileron/20.0) + 0.086 * (rudder/30.0);
        cy = cy0 + b2v* ( cyr*r + cyp*p );
        cz = cz0*(1-(beta_deg/57.3)^2)-0.19*(elevator/25.0) + cq * czp;
        cl = cl0 + dlda*daileron + dldr*drudder + b2v * ( clr*r + clp*p );
        cm = cm0 + cq * cmq + cz * (xcgr - xcg);
        cn = cn0 + dnda*daileron + dndr*drudder + 
            b2v * ( cnr*r + cnp*p ) - cy*(xcgr-xcg) * cbar/b;

end Aerodynamics;

// vim:ts=2:sw=2:expandtab
