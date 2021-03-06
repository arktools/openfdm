within F16;

model Jet

    Real idleTable =
    {{   0,       0,   10000,   20000,   30000,   40000,   50000},
      {0.0,  1060.0,   670.0,   880.0,  1140.0,  1500.0,  1860.0},
      {0.2,   635.0,   425.0,   690.0,  1010.0,  1330.0,  1700.0},
      {0.4,    60.0,    25.0,   345.0,   755.0,  1130.0,  1525.0},
      {0.6, -1020.0,  -710.0,  -300.0,   350.0,   910.0,  1360.0},
      {0.8, -2700.0, -1900.0, -1300.0,  -247.0,   600.0,  1100.0},
      {1.0, -3600.0, -1400.0,  -595.0,  -342.0,  -200.0,   700.0}}
      "idle thrust table(altitude[ft], mach number)";

    Real militaryTable =
    {{   0,       0,   10000,   20000,   30000,   40000,   50000},
      {0.0, 12680.0,  9150.0,  6200.0,  3950.0,  2450.0,  1400.0},
      {0.2, 12680.0,  9150.0,  6313.0,  4040.0,  2470.0,  1400.0},
      {0.4, 12610.0,  9312.0,  6610.0,  4290.0,  2600.0,  1560.0},
      {0.6, 12640.0,  9839.0,  7090.0,  4660.0,  2840.0,  1660.0},
      {0.8, 12390.0, 10176.0,  7750.0,  5320.0,  3250.0,  1930.0},
      {1.0, 11680.0,  9848.0,  8050.0,  6100.0,  3800.0,  2310.0}}
      "military thrust table(altitude[ft], mach number)";

    Real maximumTable =
    {{   0,       0,   10000,   20000,   30000,   40000,   50000},
      {0.0, 20000.0, 15000.0, 10800.0,  7000.0,  4000.0,  2500.0},
      {0.2, 21420.0, 15700.0, 11225.0,  7323.0,  4435.0,  2600.0},
      {0.4, 22700.0, 16860.0, 12250.0,  8154.0,  5000.0,  2835.0},
      {0.6, 24240.0, 18910.0, 13760.0,  9285.0,  5700.0,  3215.0},
      {0.8, 26070.0, 21075.0, 15975.0, 11115.0,  6860.0,  3950.0},
      {1.0, 28886.0, 23319.0, 18300.0, 13484.0,  8642.0,  5057.0}}
      "maximum thrust table(altitude[ft], mach number)";

end Jet;
