! Name : f16.f90
! Purpose : Model nonlinear f16 dynamics
! Author: James Goppert

! These functions have been adapted from
! Aircraft Control and Simulation, Stevens/Lewis 2003
! atmosphere model 

module f16

    ! using lbf(F)-slug(M)-ft(L)-sec(T) unit system

    use aircraft

    implicit none

    save ! make sure no variables get wiped out

    contains

    function f16_dynamics() result(state_dot)
        real, dimension(13) :: state_dot 
        state_dot = aircraft_dynamics(standard_atmosphere,f16_engine,f16_aerodynamics) 
    end function f16_dynamics

    subroutine f16_init
        real, parameter :: xcg = 0.35
        real, parameter :: s = 300              ! planform area, L^2
        real, parameter :: b = 30               ! wing span, L
        real, parameter :: cbar = 11.32         ! mean chord length, L
        real, parameter :: weight = 20490.446   ! weight of aircraft, lbf
        real, parameter :: axx = 9496.0         ! inertia about x axis @cm, M-L^2
        real, parameter :: ayy = 55814.0        ! inertia about y axis @cm, M-L^2
        real, parameter :: azz = 63100.0        ! inertia about z axis @cm, M-L^2
        real, parameter :: axz = 982.0          ! inertia xz term about @cm
        real, parameter :: hx = 160.0           ! x-axis engine angular momentum, M-L^2
        real, parameter :: xcgr = 0.35          ! aerodynamic reference, % chord
        call aircraft_init(s,b,cbar,weight,axx,ayy,azz,axz,hx,xcg,xcgr)
    end subroutine f16_init

    function f16_trim_cost(x) result(cost)
        real, dimension(:), intent(in)  :: x
        real, dimension(13) :: dx
        real :: cost
        throttle = x(1)
        elevator = x(2)
        alpha = x(3)
        aileron = x(4)
        rudder = x(5)
        beta = x(6)
        power = f16_throttle_gearing(throttle)
        call aircraft_constraint
        dx = f16_dynamics()
        cost = dx(1)**2 + 100.0*( dx(2)**2 + dx(3)**2 ) + &
            10.0*( dx(7)**2 + dx(8)**2 + dx(9)**2 )
    end function f16_trim_cost

 
    ! power command vs throttle relationship
    real function f16_throttle_gearing(throttle)
        implicit none
        real, intent(in) :: throttle
        if (throttle <= 0.77) then
            f16_throttle_gearing = 64.94*throttle
        else
            f16_throttle_gearing = 217.38*throttle-117.38
        end if
        return
    end function f16_throttle_gearing

    subroutine f16_engine(power_dot,thrust)
        real, intent(out) :: power_dot
        real, intent(out) :: thrust
        real :: cpow

        cpow = f16_throttle_gearing(throttle)
        power_dot = power_derivative(power,cpow)
        thrust = compute_thrust(power,alt,amach)

        contains

                ! rate of change of power
        real function power_derivative(power,power_cmd)
            implicit none
            real, intent(in) :: power       ! engine power
            real, intent(in) :: power_cmd   ! command engine power
            real :: p                       ! target power
            real :: t                       ! reciprocal of time constant, 1/T 
            if (power_cmd >= 50.0) then
                if (power >= 50.0) then
                    p = power_cmd
                    t = 5.0
                else
                    p = 60.0
                    t = rtau(p-power)
                end if
            else
                if (power >= 50.0 ) then
                    p = 40.0
                    t = 5.0
                else
                    p = power_cmd
                    t = rtau(p-power)
                end if
            end if
            power_derivative=t*(p-power)
            return
        end function power_derivative

        ! compute reciprocal of time constant for engine
        real function rtau(dp)
            implicit none
            real :: dp      ! power difference
            if (dp <= 25.0) then
                rtau = 1.0
            else if (dp >= 50.0) then
                rtau = 0.1
            else
                rtau = 1.9-0.036*dp
            end if
            return
        end function rtau

        ! engine thrust model
        real function compute_thrust(power,alt,rmach)
            implicit none
            real, intent(in) :: power       ! engine power
            real, intent(in) :: alt         ! altitude
            real, intent(in) :: rmach       ! mach number
            integer :: i, m                 ! table indices
            real :: t_idle, t_military, t_maximum   ! temporaries
            real :: idle_table(0:5,0:5), military_table(0:5,0:5), maximum_table(0:5,0:5)
            data idle_table/ &
            !                       altitude(i), ft
            !       0       10k       20k     30k      40k      50k
            !------------------------------------------------------------------
                  1060.0,   670.0,   880.0,  1140.0,  1500.0,  1860.0, & ! 0.0
                   635.0,   425.0,   690.0,  1010.0,  1330.0,  1700.0, & ! 0.2
                    60.0,    25.0,   345.0,   755.0,  1130.0,  1525.0, & ! 0.4
                 -1020.0,  -710.0,  -300.0,   350.0,   910.0,  1360.0, & ! 0.6 mach
                 -2700.0, -1900.0, -1300.0,  -247.0,   600.0,  1100.0, & ! 0.8 number(j)
                 -3600.0, -1400.0,  -595.0,  -342.0,  -200.0,   700.0/   ! 1.0

            !                       altitude(i), ft
            !       0       10k       20k     30k      40k      50k
            !------------------------------------------------------------------
            data military_table/ &
                 12680.0,  9150.0,  6200.0,  3950.0,  2450.0,  1400.0, & ! 0.0
                 12680.0,  9150.0,  6313.0,  4040.0,  2470.0,  1400.0, & ! 0.2
                 12610.0,  9312.0,  6610.0,  4290.0,  2600.0,  1560.0, & ! 0.4
                 12640.0,  9839.0,  7090.0,  4660.0,  2840.0,  1660.0, & ! 0.6 mach
                 12390.0, 10176.0,  7750.0,  5320.0,  3250.0,  1930.0, & ! 0.8 number(j)
                 11680.0,  9848.0,  8050.0,  6100.0,  3800.0,  2310.0/   ! 1.0
            !                       altitude(i), ft
            !       0       10k       20k     30k      40k      50k
            !------------------------------------------------------------------
            data maximum_table/ &
                 20000.0, 15000.0, 10800.0,  7000.0,  4000.0,  2500.0, & ! 0.0
                 21420.0, 15700.0, 11225.0,  7323.0,  4435.0,  2600.0, & ! 0.2
                 22700.0, 16860.0, 12250.0,  8154.0,  5000.0,  2835.0, & ! 0.4
                 24240.0, 18910.0, 13760.0,  9285.0,  5700.0,  3215.0, & ! 0.6 mach
                 26070.0, 21075.0, 15975.0, 11115.0,  6860.0,  3950.0, & ! 0.8 number(j)
                 28886.0, 23319.0, 18300.0, 13484.0,  8642.0,  5057.0/   ! 1.0

            t_idle = interp2d(idle_table,alt,10000.0,0.0,rmach,0.2,0.0)
            t_military = interp2d(military_table,alt,10000.0,0.0,rmach,0.2,0.0)
            t_maximum = interp2d(maximum_table,alt,10000.0,0.0,rmach,0.2,0.0)

            if ( power .lt. 50.0) then
                compute_thrust = t_idle + (t_military-t_idle) * power * .02
            else
                compute_thrust = t_military + (t_maximum-t_military) * (power-50.0) * .02
            end if

            return
        end function compute_thrust

    end subroutine f16_engine

    subroutine f16_aerodynamics(cx, cy, cz, cl, cm, cn)
        real, intent(out) :: cx, cy, cz, cl, cm, cn
        real :: alpha_deg, beta_deg, &
            b2v, cq, daileron, drudder, tvt, cxq, cyr, cyp, czp, clr, clp, &
            cmq, cnr, cnp, cx0, cy0, cz0, cl0, cm0, cn0, dlda, dldr, dnda, &
            dndr

        ! note tabes are in col major order
        real :: damping_table(-2:9,9)
        data damping_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
            -0.267, -0.110,  0.308,   1.34,   2.08,   2.91,   2.76,   2.05,   1.50,   1.49,   1.83,   1.21, & ! cxq
             0.882,  0.852,  0.876,  0.958,  0.962,  0.974,  0.819,  0.483,  0.590,  0.121, -0.493,  -1.04, & ! cyr
            -0.108, -0.108, -0.188,  0.110,  0.258,  0.226,  0.344,  0.362,  0.611,  0.529,  0.298,  -2.27, & ! cyp
             -8.80,  -25.8,  -28.9,  -31.4,  -31.2,  -30.7,  -27.7,  -28.2,  -29.0,  -29.8,  -38.3,  -35.3, & ! czp
            -0.126, -0.026,  0.063,  0.113,  0.208,  0.230,  0.319,  0.437,  0.680,  0.100,  0.447, -0.330, & ! clr  damping 
            -0.360, -0.359, -0.443, -0.420, -0.383, -0.375, -0.329, -0.294, -0.230, -0.210, -0.120, -0.100, & ! clp  coeff(j)
             -7.21, -0.540,  -5.23,  -5.26,  -6.11,  -6.64,  -5.69,  -6.00,  -6.20,  -6.40,  -6.60,  -6.00, & ! cmp
            -0.380, -0.363, -0.378, -0.386, -0.370, -0.453, -0.550, -0.582, -0.595, -0.637,  -1.02, -0.840, & ! cnr
             0.061,  0.052,  0.052, -0.012, -0.013, -0.024,  0.050,  0.150,  0.130,  0.158,  0.240,  0.150/   ! cnp

        real :: cx_table(-2:9,-2:2) 
        data cx_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
            -0.099, -0.081, -0.081,  0.063, -0.025,  0.044,  0.097,  0.113,  0.145,  0.167,  0.174,  0.166, & ! -24
            -0.048,  0.038, -0.040, -0.021,  0.016,  0.083,  0.127,  0.137,  0.162,  0.177,  0.179,  0.167, & ! -12
            -0.022, -0.020, -0.021, -0.004,  0.032,  0.094,  0.128,  0.130,  0.154,  0.161,  0.155,  0.138, & !  -0 elevator
            -0.040, -0.038, -0.039, -0.025,  0.006,  0.062,  0.087,  0.085,  0.100,  0.110,  0.104,  0.091, & !  12 (j)     
            -0.083, -0.073, -0.076, -0.072, -0.046,  0.012,  0.024,  0.025,  0.043,  0.053,  0.047,  0.040/   !  25      deg           

        real :: cz_table(-2:9) 
        data cz_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
             0.770,  0.241, -0.100, -0.416, -0.731, -1.053, -1.366, -1.646, -1.917, -2.120, -2.248, -2.229/   ! cz

        real :: cm_table(-2:9,-2:2) 
        data cm_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
             0.205,  0.168,  0.186,  0.196,  0.213,  0.251,  0.245,  0.238,  0.252,  0.231,  0.198,  0.192,  & ! -24
             0.081,  0.077,  0.107,  0.110,  0.110,  0.141,  0.127,  0.119,  0.133,  0.108,  0.081,  0.093,  & ! -12 elevator(j),
            -0.046, -0.020, -0.009, -0.005, -0.006,  0.010,  0.006, -0.001,  0.014,  0.000, -0.013,  0.032,  & !   0      deg
            -0.174, -0.145, -0.121, -0.127, -0.129, -0.102, -0.097, -0.113, -0.087, -0.084, -0.069, -0.006,  & !  12
            -0.259, -0.202, -0.184, -0.193, -0.199, -0.150, -0.160, -0.167, -0.104, -0.076, -0.041, -0.005/    !  24

        real :: cl_table(-2:9,0:6) 
        data cl_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
             0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, &  !   0
            -0.001, -0.004, -0.008, -0.012, -0.016, -0.019, -0.020, -0.020, -0.015, -0.008, -0.013, -0.015, &  !   5  (+/-) 
            -0.003, -0.009, -0.017, -0.024, -0.030, -0.034, -0.040, -0.037, -0.016, -0.002, -0.010, -0.019, &  !  10 beta(j),
            -0.001, -0.010, -0.020, -0.030, -0.039, -0.044, -0.050, -0.049, -0.023, -0.006, -0.014, -0.027, &  !  15  deg
             0.000, -0.010, -0.022, -0.034, -0.047, -0.046, -0.059, -0.061, -0.033, -0.036, -0.035, -0.035, &  !  20
             0.007, -0.010, -0.023, -0.034, -0.049, -0.046, -0.068, -0.071, -0.060, -0.058, -0.062, -0.059, &  !  25
             0.009, -0.011, -0.023, -0.037, -0.050, -0.047, -0.074, -0.079, -0.091, -0.076, -0.077, -0.076/    !  30           

        real :: cn_table(-2:9,0:6) 
        data cn_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
             0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, &  !   0
             0.018,  0.019,  0.018,  0.019,  0.019,  0.018,  0.013,  0.007,  0.004, -0.014, -0.017, -0.033, &  !   5
             0.038,  0.042,  0.042,  0.042,  0.043,  0.039,  0.030,  0.017,  0.004, -0.035, -0.047, -0.057, &  !  10  (+/-)
             0.056,  0.057,  0.059,  0.058,  0.058,  0.053,  0.032,  0.012,  0.002, -0.046, -0.071, -0.073, &  !  15 beta(i),
             0.064,  0.077,  0.076,  0.074,  0.073,  0.057,  0.029,  0.007,  0.012, -0.034, -0.065, -0.041, &  !  20  deg
             0.074,  0.086,  0.093,  0.089,  0.080,  0.062,  0.049,  0.022,  0.028, -0.012, -0.002, -0.013, &  !  25
             0.079,  0.090,  0.106,  0.106,  0.096,  0.080,  0.068,  0.030,  0.064,  0.015,  0.011, -0.001/    !  30

        real :: dlda_table(-2:9,-3:3)
        data dlda_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
            -0.041, -0.052, -0.053, -0.056, -0.050, -0.056, -0.082, -0.059, -0.042, -0.038, -0.027, -0.017, & ! -30
            -0.041, -0.053, -0.053, -0.053, -0.050, -0.051, -0.066, -0.043, -0.038, -0.027, -0.023, -0.016, & ! -20
            -0.042, -0.053, -0.052, -0.051, -0.049, -0.049, -0.043, -0.035, -0.026, -0.016, -0.018, -0.014, & ! -10
            -0.040, -0.052, -0.051, -0.052, -0.048, -0.048, -0.042, -0.037, -0.031, -0.026, -0.017, -0.012, & !   0 beta(j),
            -0.043, -0.049, -0.048, -0.049, -0.043, -0.042, -0.042, -0.036, -0.025, -0.021, -0.016, -0.011, & !  10  deg
            -0.044, -0.048, -0.048, -0.047, -0.042, -0.041, -0.020, -0.028, -0.013, -0.014, -0.011, -0.010, & !  20
            -0.043, -0.049, -0.047, -0.045, -0.042, -0.037, -0.003, -0.013, -0.010, -0.003, -0.007, -0.008/   !  30

        real :: dldr_table(-2:9,-3:3)
        data dldr_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
             0.005,  0.017,  0.014,  0.010, -0.005,  0.009,  0.019,  0.005, -0.000, -0.005, -0.011,  0.008, & ! -30
             0.007,  0.016,  0.014,  0.014,  0.013,  0.009,  0.012,  0.005,  0.000,  0.004,  0.009,  0.007, & ! -20
             0.013,  0.013,  0.011,  0.012,  0.011,  0.009,  0.008,  0.005, -0.002,  0.005,  0.003,  0.005, & ! -10
             0.018,  0.015,  0.015,  0.014,  0.014,  0.014,  0.014,  0.015,  0.013,  0.011,  0.006,  0.001, & !   0 beta(j),
             0.015,  0.014,  0.013,  0.013,  0.012,  0.011,  0.011,  0.010,  0.008,  0.008,  0.007,  0.003, & !  10 deg
             0.021,  0.011,  0.010,  0.011,  0.010,  0.009,  0.008,  0.010,  0.006,  0.005,  0.000,  0.001, & !  20
             0.023,  0.010,  0.011,  0.011,  0.011,  0.010,  0.008,  0.010,  0.006,  0.014,  0.020,  0.000/   !  30

        real :: dnda_table(-2:9,-3:3)
        ! i: alpha, j: beta
        data dnda_table/ &
        !                                                   alpha(i), deg
        !     -10     -5        0       5      10      15     20       25      30      35      40     45  
        !-----------------------------------------------------------------------------------------------------------------
             0.001, -0.027, -0.017, -0.013, -0.012, -0.016,  0.001,  0.017,  0.011,  0.017,  0.008,  0.016, & ! -30
             0.002, -0.014, -0.016, -0.016, -0.014, -0.019, -0.021,  0.002,  0.012,  0.015,  0.015,  0.011, & ! -20
            -0.006, -0.008, -0.006, -0.006, -0.005, -0.008, -0.005,  0.007,  0.004,  0.007,  0.006,  0.006, & ! -10
            -0.011, -0.011, -0.010, -0.009, -0.008, -0.006,  0.000,  0.004,  0.007,  0.010,  0.004,  0.010, & !   0 beta(j),
            -0.015, -0.015, -0.014, -0.012, -0.011, -0.008, -0.002,  0.002,  0.006,  0.012,  0.011,  0.011, & !  10 deg
            -0.024, -0.010, -0.004, -0.002, -0.001,  0.003,  0.014,  0.006, -0.001,  0.004,  0.004,  0.006, & !  20
            -0.022,  0.002, -0.003, -0.005, -0.003, -0.001, -0.009, -0.009, -0.001,  0.003, -0.002,  0.001/   !  30

        real :: dndr_table(-2:9,-3:3)
        data dndr_table/ &
        !                                                   alpha(i), deg
        !     -10       -5      0       5      10      15     20       25      30      35      40     45  
        !-------------------------------------------------------------------------------------------------------------------
            -0.018, -0.052, -0.052, -0.052, -0.054, -0.049, -0.059, -0.051, -0.030, -0.037, -0.026, -0.013, & ! -30
            -0.028, -0.051, -0.043, -0.046, -0.045, -0.049, -0.057, -0.052, -0.030, -0.033, -0.030, -0.008, & ! -20
            -0.037, -0.041, -0.038, -0.040, -0.040, -0.038, -0.037, -0.030, -0.027, -0.024, -0.019, -0.013, & ! -10
            -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047, -0.048, -0.049, -0.045, -0.033, -0.016, & !   0 beta(j), 
            -0.043, -0.044, -0.041, -0.041, -0.040, -0.038, -0.034, -0.035, -0.035, -0.029, -0.022, -0.009, & !  10 deg
            -0.052, -0.034, -0.036, -0.036, -0.035, -0.028, -0.024, -0.023, -0.020, -0.016, -0.010, -0.014, & !  20
            -0.062, -0.034, -0.027, -0.028, -0.027, -0.027, -0.023, -0.023, -0.019, -0.009, -0.025, -0.010/   !  30

        ! prepare for look up tables
        alpha_deg = alpha*rtod;
        beta_deg = beta*rtod;

        ! force coeff
        cx0 = interp2d(cx_table,alpha_deg,5.0,-10.0,elevator,12.0,-24.0)
        cy0 = -.02*beta_deg + .021*(aileron/20.0) + .086 * (rudder/30.0)
        cz0 = interp1d(cz_table,alpha_deg,5.0,-10.0)*(1-(beta_deg/57.3)**2) &
            -.19*(elevator/25.0)

        ! moment coeff
 
        cl0 = sign(1.0,beta_deg)*interp2d(cl_table,alpha_deg,5.0,-10.0,abs(beta_deg),5.0,0.0)
        cm0 = interp2d(cm_table,alpha_deg,5.0,-10.0,elevator,12.0,-24.0)
        cn0 = sign(1.0,beta_deg)*interp2d(cn_table,alpha_deg,5.0,-10.0,abs(beta_deg),5.0,0.0)

        ! aileron effects
        dnda = interp2d(dnda_table,alpha_deg,5.0,-10.0,beta_deg,10.0,-30.0)
        dlda = interp2d(dlda_table,alpha_deg,5.0,-10.0,beta_deg,10.0,-30.0)

        ! rudder effects
        dldr = interp2d(dldr_table,alpha_deg,5.0,-10.0,beta_deg,10.0,-30.0)
        dndr = interp2d(dndr_table,alpha_deg,5.0,-10.0,beta_deg,10.0,-30.0)

        ! damping coeff
        cxq = interp2d(damping_table,alpha_deg,5.0,-10.0,1.0,1.0,1.0)
        cyr = interp2d(damping_table,alpha_deg,5.0,-10.0,2.0,1.0,1.0)
        cyp = interp2d(damping_table,alpha_deg,5.0,-10.0,3.0,1.0,1.0)
        czp = interp2d(damping_table,alpha_deg,5.0,-10.0,4.0,1.0,1.0)
        clr = interp2d(damping_table,alpha_deg,5.0,-10.0,5.0,1.0,1.0)
        clp = interp2d(damping_table,alpha_deg,5.0,-10.0,6.0,1.0,1.0)
        cmq = interp2d(damping_table,alpha_deg,5.0,-10.0,7.0,1.0,1.0)
        cnr = interp2d(damping_table,alpha_deg,5.0,-10.0,8.0,1.0,1.0)
        cnp = interp2d(damping_table,alpha_deg,5.0,-10.0,9.0,1.0,1.0)

        ! compute coefficients
        tvt = .5/vt
        b2v = b*tvt
        cq = cbar*q*tvt
        daileron = aileron/20.0;
        drudder = rudder/30.0;

        cx = cx0 + cq * cxq
        cy = cy0 + b2v* ( cyr*r + cyp*p )
        cz = cz0 + cq * czp
        cl = cl0 + dlda*daileron + dldr*drudder + b2v * ( clr*r + clp*p )
        cm = cm0 + cq * cmq + cz * (xcgr - xcg)
        cn = cn0 + dnda*daileron + dndr*drudder + &
            b2v * ( cnr*r + cnp*p ) - cy*(xcgr-xcg) * cbar/b

    end subroutine f16_aerodynamics

end module f16
