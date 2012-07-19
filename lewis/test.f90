program f16_trim
    use f16
    implicit none

    real, dimension(6) :: x, h0
    data x/0,0,0,0,0,0/
    data h0/.2,1.0,.02,1.0,1.0,.02/

    ! defaults
    call f16_init
    vt = 502.0
    xcg = 0.35

    call aircraft_trim(x,h0,f16_trim_cost,f16_dynamics)

end program f16_trim
