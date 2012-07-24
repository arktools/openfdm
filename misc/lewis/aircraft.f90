! Name : aircraft.f90
! Purpose : Model nonlinear aircraft dynamics
! Author: James Goppert

! These functions have been adapted from
! Aircraft Control and Simulation, Stevens/Lewis 2003

module aircraft

    use math

    implicit none

    save ! make sure no variables get wiped out

    external atmosphere

    ! enumerations for state
    integer, parameter :: e_vt=1, e_alpha=2, e_beta=3, e_phi=4, e_theta=5, &
        e_psi=6, e_p=7, e_q=8, e_r=9, e_pos_north=10, e_pos_east=11, &
        e_alt=12, e_power=13

    ! enumerations for inputs
    integer, parameter :: e_throttle=1, e_elevator=2, e_aileron=3, e_rudder=4

    ! enumerations for outputs
    integer, parameter :: e_an=1, e_alat=2, e_ax=4, e_qbar=5, e_amach=6

    ! physical attributes
    real :: s           ! planform area, L^2
    real :: b           ! wing span, L
    real :: cbar        ! mean chord length, L
    real :: weight      ! weight of aircraft, M
    real :: axx         ! inertia about x axis @cm, M-L^2
    real :: ayy         ! inertia about y axis @cm, M-L^2
    real :: azz         ! inertia about z axis @cm, M-L^2
    real :: axz         ! inertia xz term about @cm
    real :: hx          ! x-axis engine angular momentum, M-L^2
    real :: xcg         ! center of gravity, % chord
    real :: xcgr        ! aerodynamic reference point, % chord

    ! precomputed inertia terms
    real :: mass        ! mass, M
    real :: axzs        ! M^2-L^4   
    real :: xpq
    real :: zeta
    real :: xqr
    real :: zpq 
    real :: ypr

    ! control variables
    real :: throttle
    real :: elevator
    real :: aileron
    real :: rudder

    ! states  
    real :: vt          !  1  true velocity, L/T
    real :: alpha       !  2  angle of attack, rad 
    real :: beta        !  3  side slip angle, rad
    real :: phi         !  4  roll angle, rad
    real :: theta       !  5  pitch angle, rad
    real :: psi         !  6  heading angle, rad
    real :: p           !  7  body roll rate, rad/T
    real :: q           !  8  pitch rate, rad/T
    real :: r           !  9  body yaw rate, L/T
    real :: pos_north   ! 10  north position, L 
    real :: pos_east    ! 11  east position, L 
    real :: alt         ! 12  altitude, L 
    real :: power       ! 13  engine poiwer, L-F/T

    ! output
    real :: an                  ! north acceleration, g's
    real :: alat                ! east acceleration, g's
    real :: ax                  ! ? not used ?
    real :: qbar                ! dynamic pressure, F/L^2
    real :: amach               ! mach number

    ! constraint related
    real :: gam                     ! flight path angle
    real :: roll_rate               ! roll rate
    real :: pitch_rate              ! pitch rate of aircraft
    real :: yaw_rate                ! yaw rate of aircraft
    logical :: coordinated_turn     ! whether or not the turn is coordinated
    logical :: stab_axis_roll       

    contains

    subroutine aircraft_init(s_p,b_p,cbar_p,weight_p,axx_p,ayy_p,azz_p,&
        axz_p,hx_p,xcg_p,xcgr_p)
        real, intent(in) :: s_p, b_p, cbar_p, weight_p, &
            axx_p, ayy_p, azz_p, axz_p, hx_p, xcg_p, xcgr_p

        ! parameters
        s = s_p
        b = b_p
        cbar = cbar_p
        weight = weight_p
        axx = axx_p
        ayy = ayy_p
        azz = azz_p
        axz = axz_p
        hx = hx_p
        xcg = xcg_p
        xcgr = xcgr_p
        mass = weight/gd
        axzs = axz**2
        xpq = axz*(axx-ayy+azz)
        zeta = axx*azz-axz**2     
        xqr = azz*(azz-ayy)+axzs 
        zpq = (axx-ayy)*axx+axzs 
        ypr = azz-axx 

        ! control
        throttle = 0.0
        elevator = 0.0
        aileron = 0.0
        rudder = 0.0

        ! states
        vt = 0.0 
        alpha = 0.0
        beta = 0.0 
        phi = 0.0 
        theta = 0.0 
        psi = 0.0  
        p = 0.0 
        q = 0.0    
        r = 0.0    
        alt  = 0.0       
        pos_north = 0.0
        pos_east = 0.0 
        power = 0.0 

        ! output
        an = 0.0
        alat = 0.0
        ax = 0.0
        qbar = 0.0
        amach = 0.0

        ! constraint related
        stab_axis_roll = .true.

    end subroutine aircraft_init        

    subroutine aircraft_load_state(x)
        real, intent(in), dimension(13) :: x
        vt = x(e_vt)
        alpha = x(e_alpha)
        beta = x(e_beta)
        phi = x(e_phi)
        theta = x(e_theta)
        psi = x(e_psi)
        p = x(e_p)
        q = x(e_q)
        r = x(e_r)
        pos_north = x(e_pos_north)
        pos_east = x(e_pos_east)
        alt = x(e_alt)
        power = x(e_power)
    end subroutine aircraft_load_state

    function aircraft_get_state() result(x)
        real, dimension(13) :: x
        x(e_vt) = vt
        x(e_alpha) = alpha
        x(e_beta) = beta 
        x(e_phi) = phi
        x(e_theta) = theta
        x(e_psi) = psi
        x(e_p) = p
        x(e_q) = q
        x(e_r) = r
        x(e_pos_north) = pos_north
        x(e_pos_east) = pos_east
        x(e_alt) = alt
        x(e_power) = power
    end function aircraft_get_state

    function aircraft_dynamics(atmosphere,engine,aerodynamics) result(state_dot)
        real, dimension(13) :: state_dot        ! aircraft derivative

        ! subroutines
        interface
            subroutine atmosphere(amach,qbar)
                real, intent(out) :: amach          ! mach number
                real, intent(out) :: qbar           ! dynamic pressure, F/L^2
            end subroutine atmosphere
            subroutine engine(power_dot,thrust)
                real, intent(out) :: power_dot      ! power derivative F-L/T^2
                real, intent(out) :: thrust         ! thrust of engine
            end subroutine engine
            subroutine aerodynamics(cx, cy, cz, cl, cm, cn)
                real, intent(out) :: cx, cy, cz     ! force coeff.
                real, intent(out) :: cl, cm, cn     ! moment coeff.
            end subroutine
        end interface

        ! subroutine temporaries
        real :: power_dot
        real :: thrust              ! thrust, F 
        real :: cx, cy, cz          ! force coefficients
        real :: cl, cm, cn          ! moment coefficients

        ! precomputed state equation terms
        real :: c_beta, u, v, w, s_theta, c_theta, s_phi, c_phi, &
            s_psi, c_psi, qs, qsb, rmqs, g_c_theta, q_s_phi, ay, &
            az, pq, qr, qhx, t1, t2, t3, s1, s2, s3, s4, s5, s6, &
            s7, s8 , roll_m, pitch_m, yaw_m, dum, udot, vdot, wdot, &
            tvt, b2v, cq
        
        ! compute atmosphere variables
        call atmosphere(amach,qbar)

        ! engine model
        call engine(power_dot,thrust)
        state_dot(13) = power_dot

        ! aerodynamics 
        call aerodynamics(cx, cy, cz, cl, cm, cn)

        ! precomputation for state equations
        c_beta = cos(beta)
        u = vt*cos(alpha)*c_beta
        v = vt*sin(beta)
        w = vt*sin(alpha)*c_beta
        s_theta = sin(theta)
        c_theta = cos(theta)
        s_phi = sin(phi)
        c_phi = cos(phi)
        s_psi = sin(psi)
        c_psi = cos(psi)
        qs = qbar * s
        qsb = qs * b
        rmqs = qs/mass
        g_c_theta = gd * c_theta
        q_s_phi = q * s_phi 
        ay = rmqs * cy
        az = rmqs * cz

        ! force equations
        udot = r*v - q*w - gd * s_theta + (qs * cx + thrust)/mass
        vdot = p*w - r*u + g_c_theta * s_phi + ay
        wdot = q*u - p*v + g_c_theta * c_phi + az
        dum = u*u + w*w
        state_dot(1) = (u*udot + v*vdot + w*wdot)/vt
        state_dot(2) = (u*wdot - w*udot) / dum
        state_dot(3) = (vt*vdot - v*state_dot(1)) * c_beta / dum

        ! kinematics
        state_dot(4) = p + (s_theta/c_theta)*(q_s_phi + r*c_phi)
        state_dot(5) = q*c_phi - r*s_phi
        state_dot(6) = (q_s_phi + r*c_phi)/c_theta

        ! moments
        roll_m = qsb*cl
        pitch_m = qs * cbar * cm
        yaw_m = qsb*cn
        pq = p*q
        qr = q*r
        qhx = q*hx
        state_dot(7) = ( xpq*pq - xqr*qr + azz*roll_m + axz*(yaw_m + qhx) ) /zeta
        state_dot(8) = ( ypr*p*r - axz*(p**2 - r**2) +&
             pitch_m - r*hx ) / ayy
        state_dot(9) = ( zpq*pq - xpq*qr + axz*roll_m + axx*(yaw_m + qhx) ) /zeta

        ! navigation
        t1 = s_phi * c_psi
        t2 = c_phi * s_theta
        t3 = s_phi * s_psi
        s1 = c_theta * c_psi
        s2 = c_theta * s_psi
        s3 = t1 * s_theta - c_phi * s_psi
        s4 = t3 * s_theta + c_phi * c_psi
        s5 = s_phi * c_theta
        s6 = t2*c_phi + t3
        s7 = t2 * s_psi - t1
        s8 = c_phi * c_theta
        state_dot(10) = u * s1 + v * s3 + w * s6 ! north speed
        state_dot(11) = u * s2 + v * s4 + w * s7 ! east speed
        state_dot(12) = u * s_theta - v * s5 - w * s8 ! vertical speed

        ! outputs
        an = -az/gd
        alat = ay/gd
        
    end function aircraft_dynamics

    subroutine aircraft_constraint
        real :: a, b, c, gc, &
            s_gam, s_beta, c_beta, t_alpha, &
            c_alpha

        ! precomputation
        s_gam = sin(gam)
        s_beta = sin(beta)
        c_beta = cos(beta)
        t_alpha = tan(alpha)
        c_alpha = cos(alpha)

        ! turn coordination constraint, lewis pg. 190
        gc = yaw_rate*vt/gd;
        a = 1 - gc*t_alpha*s_beta
        b = s_gam/c_beta
        c = 1 + gc*gc*c_beta*c_beta
        phi = atan((gc*c_beta*(a-b*b)+&
            b*t_alpha*sqrt(c*(1-b*b)+gc*gc*s_beta*s_beta))/ &
            (c_alpha*(a*a-b*b*(1+c*t_alpha*t_alpha))))

        ! rate of climb constraint, lewis pg. 189
        a = c_alpha*c_beta
        b = sin(phi)*s_beta+cos(phi)*sin(alpha)*c_beta
        theta = atan((a*b+s_gam*sqrt(a*a-s_gam*s_gam+b*b))/&
            (a*a-s_gam*s_gam))

        ! rolling
        if (roll_rate /= 0.0) then
            p = roll_rate
            q = 0.0
            if (stab_axis_roll) then
                r = roll_rate*tan(alpha)
            else
                r = roll_rate
            end if
        ! yawing
        else if (yaw_rate /= 0.0) then
            p = -yaw_rate*sin(theta)
            q = yaw_rate*sin(phi)*cos(theta)
            r = yaw_rate*cos(phi)*cos(theta)
        ! pitching
        else if (pitch_rate /= 0.0) then
            p = 0.0
            q = pitch_rate
            r = 0.0
        end if

    end subroutine aircraft_constraint

    subroutine standard_atmosphere(amach,qbar)
        real, parameter :: rho0 = 2.377e-3  ! sea-level density, slug/ft^3
        real, intent(out) :: amach          ! mach number
        real, intent(out) :: qbar           ! dynamic pressure, F/L^2
        real :: ps                          ! static pressure, F/L^2
        real :: tfac                        ! temperature factor
        real :: t                           ! temperature, K
        real :: rho                         ! density, M/L^3
        
        tfac = 1 - 0.703e-5 * alt
        t = 519.0 * tfac                
        if (alt .ge. 35000.0) t = 390.0
        rho = rho0 * (tfac**4.14)       
        amach = vt/sqrt(1.4*1716.3*t)   
        qbar = 0.5*rho*vt*vt            
        ps = 1715.0 * rho * t           
    end subroutine standard_atmosphere

    subroutine aircraft_trim(x,h0,trim_cost,dynamics)

        ! make sure you initialize the aircraft before
        ! calling this function

        real, intent(inout),dimension(:) :: x, h0
        integer :: iter_max = 1000, choice = 0

        interface
            function trim_cost(x) result(cost)
                real, dimension(:), intent(in)  :: x
            end function
            function dynamics() result(state_dot)
                real, dimension(13) :: state_dot 
            end function
        end interface

        ! defaults
        gam = 0.0
        pitch_rate = 0.0
        yaw_rate = 0.0
        roll_rate = 0.0

        write(*,'(/1X,A/)') "Aircraft Trim Program"
        call prompt("flight path angle, deg    ",gam)
        gam = gam/rtod
        call prompt("center of gravity, %chord ",xcg)
        call prompt("altitude ,ft              ",alt)
        call prompt("air speed ,ft/s           ",vt)
        do
            call prompt("select mode, steady-level-flight(0), rolling(1), pitching(2), yawing(3): ",choice) 
            if (choice==0) then
                ! level flight
                exit
            else if (choice==1) then
                call prompt("roll rate ,rad/s          ",roll_rate)
                exit
            else if (choice==2) then
                call prompt("pitch rate ,rad/s         ",pitch_rate)
                exit
            else if (choice==3) then
                call prompt("yaw rate ,rad/s           ",yaw_rate)
                exit
            else
                write(*,*) "unknown choice: ", choice
            end if
        end do
        call prompt("iterations                ",iter_max)

        ! solve the simplex
        do
            write(*,*)
            write(*,*) "initial cost    : ", trim_cost(x)
            x = fmin(trim_cost,x,h0,1e-6,iter_max)
            ! load the solution into the module by calling trim cost again
            ! this must be called before anything else
            write(*,*) "final cost      : ", trim_cost(x)
            !write(*,*) "state_dot      : ", dynamics()
            !write(*,*) "state          : ", aircraft_get_state()
            write(*,*)
            write(*,*) "design:"
            write(*,*) "alpha, deg      : ", alpha*rtod
            write(*,*) "beta, deg       : ", beta*rtod
            write(*,*) "throttle, %     : ", throttle*100
            write(*,*) "elevator, deg   : ", elevator 
            write(*,*) "aileron, deg    : ", aileron 
            write(*,*) "rudder, deg     : ", rudder 
            write(*,*)
            write(*,*) "state:"
            write(*,*) "vt, ft/s        : ", vt 
            write(*,*) "roll, deg       : ", phi*rtod
            write(*,*) "pitch, deg      : ", theta*rtod
            write(*,*) "heading, deg    : ", psi*rtod
            write(*,*) "p, rad/s        : ", p
            write(*,*) "q, rad/s        : ", q
            write(*,*) "r, rad/s        : ", r
            write(*,*)

             call prompt("More iterations? (0 for quit)",iter_max)
            if (iter_max == 0) exit

        end do

    end subroutine aircraft_trim

end module aircraft
