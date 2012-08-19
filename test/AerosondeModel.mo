within test;

model AerosondeModel

  import C=Modelica.Constants;
  import OpenFDM.*;
  import OpenFDM.Aerodynamics.*;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;

  constant Datcom.Tables datcomTables(    
      CL_Basic =   {
        { -16,    -1.533},     
        {  -8,    -0.602},     
        {  -6,    -0.369},     
        {  -4,    -0.142},     
        {  -2,     0.083},     
        {   0,     0.315},     
        {   2,     0.552},     
        {   4,     0.795},     
        {   8,     1.288},     
        {   9,     1.401},     
        {  10,     1.503},     
        {  12,     1.685},     
        {  14,     1.832},     
        {  16,     1.933},     
        {  18,     1.892},     
        {  19,     1.589},     
        {  20,     1.226},     
        {  21,     0.891},     
        {  22,     0.646},     
        {  24,      0.18}},

      dCL_Flap  =   {    
        {   0,         0},    
        {   5,     0.086},    
        {  10,     0.172},    
        {  15,     0.254},    
        {  20,     0.332},    
        {  25,     0.407},    
        {  30,     0.468},    
        {  35,     0.515},    
        {  40,     0.547}},

      dCL_Elevator  =   {    
        { -28,    -0.142},    
        { -20,    -0.125},    
        { -10,     -0.09},    
        {  -5,    -0.045},    
        {   0,         0},    
        {   5,     0.045},    
        {  10,      0.09},    
        {  20,     0.125},    
        {  28,     0.142}},

      dCL_PitchRate =   {     
        { -16,    0.1727}},

      dCL_AlphaDot  =   {     
        { -16,   0.02933},     
        {  -8,   0.03257},     
        {  -6,   0.03381},     
        {  -4,   0.03499},     
        {  -2,   0.03694},     
        {   0,   0.03907},     
        {   2,   0.04098},     
        {   4,   0.04275},     
        {   8,   0.04536},     
        {   9,   0.03893},     
        {  10,   0.02967},     
        {  12,   0.02287},     
        {  14,   0.01428},     
        {  16, -0.009362},     
        {  18,  -0.04021},     
        {  19,   -0.0978},     
        {  20,   -0.1347},     
        {  21,   -0.1004},     
        {  22,  -0.08004},     
        {  24,  -0.07793}},

      CD_Basic  =   {     
        { -16,     0.813},     
        {  -8,     0.235},     
        {  -6,     0.151},     
        {  -4,     0.092},     
        {  -2,     0.058},     
        {   0,      0.05},     
        {   2,     0.067},     
        {   4,     0.111},     
        {   8,     0.276},     
        {   9,     0.332},     
        {  10,     0.393},     
        {  12,     0.529},     
        {  14,     0.683},     
        {  16,     0.852},     
        {  18,     1.021},     
        {  19,     1.082},     
        {  20,     1.152},     
        {  21,     1.236},     
        {  22,     1.334},     
        {  24,     1.562}},

      dCD_Flap  = empty2D,
      /*dCD_Flap  =     {*/
        /*{0,         0,         5,        10,        15,        20,        25,        30,        35,        40  },   */
        /*{ -16, -1.18e-05,  -0.00543,  -0.00986,   -0.0131,   -0.0155,   -0.0169,   -0.0176,   -0.0177,   -0.0177},   */
        /*{  -8, -4.17e-06,  -0.00159,  -0.00219,  -0.00185, -0.000688,   0.00117,   0.00326,   0.00517,   0.00668},   */
        /*{  -6, -2.25e-06,  -0.00063, -0.000272,  0.000975,   0.00301,    0.0057,   0.00847,    0.0109,    0.0128},   */
        /*{  -4, -3.29e-07,  0.000329,   0.00165,    0.0038,    0.0067,    0.0102,    0.0137,    0.0166,    0.0189},   */
        /*{  -2,  1.59e-06,   0.00129,   0.00356,   0.00662,    0.0104,    0.0147,    0.0189,    0.0223,    0.0249},   */
        /*{   0,  3.51e-06,   0.00225,   0.00548,   0.00945,    0.0141,    0.0193,    0.0241,    0.0281,     0.031},   */
        /*{   2,  5.43e-06,   0.00321,    0.0074,    0.0123,    0.0178,    0.0238,    0.0293,    0.0338,    0.0371},   */
        /*{   4,  7.35e-06,   0.00417,   0.00932,    0.0151,    0.0215,    0.0283,    0.0345,    0.0395,    0.0432},   */
        /*{   8,  1.12e-05,   0.00609,    0.0131,    0.0207,    0.0289,    0.0374,    0.0449,     0.051,    0.0554},   */
        /*{   9,  1.22e-05,   0.00657,    0.0141,    0.0222,    0.0307,    0.0396,    0.0475,    0.0538,    0.0584},   */
        /*{  10,  1.31e-05,   0.00705,    0.0151,    0.0236,    0.0326,    0.0419,    0.0501,    0.0567,    0.0614},   */
        /*{  12,   1.5e-05,     0.008,     0.017,    0.0264,    0.0363,    0.0464,    0.0553,    0.0624,    0.0675},   */
        /*{  14,  1.69e-05,   0.00896,    0.0189,    0.0292,      0.04,    0.0509,    0.0605,    0.0681,    0.0736},   */
        /*{  16,  1.89e-05,   0.00992,    0.0208,     0.032,    0.0437,    0.0555,    0.0657,    0.0738,    0.0797},   */
        /*{  18,  2.08e-05,    0.0109,    0.0227,    0.0349,    0.0474,      0.06,    0.0709,    0.0796,    0.0858},   */
        /*{  19,  2.18e-05,    0.0114,    0.0237,    0.0363,    0.0492,    0.0623,    0.0735,    0.0824,    0.0888},   */
        /*{  20,  2.27e-05,    0.0118,    0.0247,    0.0377,    0.0511,    0.0645,    0.0762,    0.0853,    0.0919},   */
        /*{  21,  2.37e-05,    0.0123,    0.0256,    0.0391,    0.0529,    0.0668,    0.0788,    0.0882,    0.0949},   */
        /*{  22,  2.47e-05,    0.0128,    0.0266,    0.0405,    0.0547,     0.069,    0.0814,     0.091,     0.098},   */
        /*{  24,  2.66e-05,    0.0138,    0.0285,    0.0433,    0.0584,    0.0736,    0.0866,    0.0967,     0.104}},*/


      dCD_Elevator  = empty2D,

      dCY_Beta  = empty1D,
      dCY_RollRate  = empty1D,

      dCl_Aileron  = empty1D,
      dCl_Beta  = empty1D,
      dCl_RollRate  = empty1D,
      dCl_YawRate  = empty1D,

      Cm_Basic = empty1D,
      /*Cm_Basic = {{-16,-1e-10},{16,1e-10}},*/
      /*Cm_Basic =   {   */
        /*{ -16,     0.434},   */
        /*{  -8,     0.277},   */
        /*{  -6,    0.1997},   */
        /*{  -4,    0.1202},   */
        /*{  -2,    0.0403},   */
        /*{   0,   -0.0397},   */
        /*{   2,   -0.1216},   */
        /*{   4,   -0.2076},   */
        /*{   8,   -0.3993},   */
        /*{   9,   -0.4548},   */
        /*{  10,   -0.5165},   */
        /*{  12,   -0.6391},   */
        /*{  14,   -0.7634},   */
        /*{  16,   -0.0153},   */
        /*{  18,   -0.0095},   */
        /*{  19,    0.0126},   */
        /*{  20,    0.1351},   */
        /*{  21,    0.1985},   */
        /*{  22,     0.155},   */
        /*{  24,    0.0624}},*/

      dCm_Flap  = empty1D,
      dCm_Elevator  = empty1D,
      dCm_PitchRate  =   {      
        { -16,   -0.7118}},
      dCm_AlphaDot  = empty1D,

      dCn_Aileron  = empty2D,
      dCn_Beta  = empty1D,
      /*dCn_Beta  =   {*/
        /*{-16, 0.0008503}},*/


      dCn_RollRate  = empty1D,
      dCn_YawRate  = empty1D);

  inner World.Earth world;

  // init aircraft in steady level flight
  // can change pitch and throttle only
  // to obtain zero flight path angle at desired vt
  Parts.RigidReferencePoint p(
    // true airspeed
    //vt(start=6,fixed=false),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={10,0,0},fixed={true,true,true}),
    // position fixed
    r_r(start={0,0,-1000},fixed={true,true,true}),
    // can change pitch, roll and heading fixed
    euler(start={0,0,0},fixed={true,false,true}),
    // no angular velocity, or acceleration
    w_ib(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    // no translational acceleration
    a_b(start={0,0,0},fixed={true,true,true}));


  model Thrust
    extends Parts.ForceMoment;
    Modelica.Blocks.Nonlinear.Limiter sat(
      uMax=1,
      uMin=0);
    input Real throttle(start=0.3,fixed=false);
  equation
    der(throttle) = 0;
    sat.u = throttle;
    F_b = sat.y*{10,0,0};
    M_b = {0,0,0};
  end Thrust;

  Datcom.ForceMoment aero(
    tables=datcomTables,
    rudder_deg = 0,
    flap_deg = 0,
    elevator_deg = 0,
    aileron_deg = 0,
    s=1, b=1, cBar=1);

  Thrust thrust;
  Parts.RigidBody structure(m=1,I_b=identity(3));
  Parts.RigidLink_B321 t_aero_rp(r_a={0,0,0}, angles={0,0,0});
  Parts.RigidLink_B321 t_motor(r_a={0,0,0}, angles={0,0,0});

equation

  assert(p.w_ib[1] < 1, "rolling too fast");
  assert(p.w_ib[2] < 1, "pitching too fast");
  assert(p.w_ib[3] < 1, "yawing too fast");
  assert(p.v_b[1] < 100, "Vx too fast");
  assert(p.v_b[2] < 100, "Vy too fast");
  assert(p.v_b[3] < 100, "Vx too fast");

  connect(p.fA,structure.fA);

  connect(p.fA,t_motor.fA);
  connect(t_motor.fB,thrust.fA);

  connect(p.fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

end AerosondeModel;

// vim:ts=2:sw=2:expandtab:
