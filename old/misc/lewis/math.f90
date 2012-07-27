module math

    implicit none

    save ! make sure no variables get wiped out

    ! physical constants
    real, parameter :: gd = 32.17        ! standard gravity, L/T^2
    real, parameter :: rtod = 57.29578  ! radian to rad. conversion

    interface prompt
        module procedure prompt_real
        module procedure prompt_integer
        module procedure prompt_logical
    end interface

    contains

    subroutine prompt_logical(str,val)
        character(*), intent(in) :: str
        logical, intent(inout) :: val
        write(*,'(1X,A,A,L12,A,$)') str, " [ ", val, " ] : "
        read(*,*) val
    end subroutine prompt_logical

    subroutine prompt_integer(str,val)
        character(*), intent(in) :: str
        integer, intent(inout) :: val
        write(*,'(1X,A,A,I12,A,$)') str, " [ ", val, " ] : "
        read(*,*) val
    end subroutine prompt_integer

    subroutine prompt_real(str,val)
        character(*), intent(in) :: str
        real, intent(inout) :: val

        write(*,'(1X,A,A,EN12.4,A,$)') str, " [ ", val, " ] : "
        read(*,*) val
    end subroutine prompt_real

    ! fourth order runge kutta algorithm
    subroutine rk4(f,h,x,y)

        interface
            ! derivative of y w.r.t. x
            function f(x,y) result (dydx)
                real, intent(in) :: x
                real, dimension(:), intent(in) :: y
                real, dimension(size(y)) :: dydx
            end function
        end interface

        real, intent(in) :: h           ! step size
        real, intent(inout) :: x        ! ind. variable, i.e. time
        real, intent(inout), dimension(:) :: y ! dep variable, i.e. state

        integer :: n, i
        real, dimension(size(y)) :: k1, k2, k3, k4, xd1, xd2, xd3, xd4

        k1 = h*f(x,y)
        k2 = h*f(x+.5*h,y+.5*k1)
        k3 = h*f(x+.5*h,y+.5*k2)
        k4 = h*f(x+h,y+k3)

        y = y + k1/6 + k2/3 + k3/3 + k4/6
        x = x + h

    end subroutine rk4

    ! display matrix
    subroutine print_mat(str,mat)
        character(len=*), intent(in) :: str
        real, dimension(:,:), intent(in) :: mat
        integer :: i_row, n_row
        n_row = size(mat,1)
        write(*,*) str
        do i_row = 1,n_row
            write(*,*) mat(i_row,:)
        end do
    end subroutine print_mat

    ! constrain to range
    subroutine limit(x,min_val,max_val)
        real, intent(inout) :: x
        real, intent(in) :: min_val, max_val
        if (x<min_val) x = min_val
        if (x>max_val) x = max_val
    end subroutine

    ! nelder mead simplex algorithm
    function fmin(f,x0,h0,rtol,iter_max) result(x)
        
        interface
            function f(x) result(cost)
                real, dimension(:), intent(in)  :: x
                real :: cost
            end function f 
        end interface

        real, dimension(:), intent(in) :: x0                ! initial state
        real, dimension(:), intent(in) :: h0                ! initial step
        real, intent(in) :: rtol                            ! relative tol.  
        integer, intent(in) :: iter_max                     ! max iterations 
        real, dimension(size(x0)) :: x                      ! solution 
        real, dimension(size(x0)) :: comp_sum               ! simplex comp. sum 
        real, dimension(size(x0),size(x0)+1) :: simplex     ! the simplex
        real, dimension(size(x0)+1) :: cost                 ! vertex costs 

        integer :: i_vertex, i_dim, n_dim, n_vert
        integer :: i_max, i_next_max, i_min
        integer :: iter

        real :: cost_try, rtol_i

        n_dim = size(x0)
        n_vert = size(x0)+1

        ! create simplex
        do i_vertex = 1,n_vert
            do i_dim = 1,n_dim
                simplex(i_dim, i_vertex) = x0(i_dim)
                if (i_vertex == i_dim + 1) then
                    simplex(i_dim,i_vertex) = simplex(i_dim,i_vertex) + h0(i_dim)
                end if
            end do
        end do

        ! initialize loop 
        i_max = 1
        i_next_max = 1
        i_min = 1
        iter = 0

        ! solve simplex
        do
            ! find cost of vertices
            do i_vertex = 1,n_vert 
                cost(i_vertex) = f( simplex(:,i_vertex) )
            end do

            ! find max cost, next max cost, and min cost
            do i_vertex=1,n_vert
                if (cost(i_vertex) > cost(i_max)) then
                    i_next_max = i_max
                    i_max = i_vertex
                else if (cost(i_vertex) < cost(i_min)) then
                    i_min = i_vertex
                end if
            end do

            ! compute fraction range
            rtol_i = 2*abs(cost(i_max)-cost(i_min))/&
                (abs(cost(i_max)+abs(cost(i_min))+epsilon(rtol)))

            ! check for termination conditions
            if (iter > iter_max) then
                write(*,*) "max iterations exceeded"
                exit
            else if (rtol_i < rtol) then
                write(*,*) "simplex converged"
                exit
            end if

            ! compute element sum of simplex vertices
            do i_dim=1,n_dim
                comp_sum(i_dim) = 0
                do i_vertex = 1,n_vert
                    comp_sum(i_dim) = comp_sum(i_dim) + simplex(i_dim,i_vertex) 
                end do
            end do

            ! try stretch by -1, invert max vertex about min face
            cost_try = try_stretch(-1.0)

            ! if lower cost, then try further stretch by 2
            if ( cost_try < cost(i_max) ) then
                 cost_try = try_stretch(2.0)    

            ! if worse than the next maximum then contract
            else
                cost_try = try_stretch(0.5)
            end if

            ! step loop
            iter = iter + 1

        end do

        ! store solution
        x = simplex(:,i_min)

        contains

        function try_stretch(factor) result(cost_try)
            real, intent(in) :: factor
            real :: a, b
            real, dimension(n_dim) :: try_vertex
            integer :: i_dim
            real :: cost_try
            a = (1.0-factor)/n_dim
            b = a - factor
            do i_dim=1,n_dim
                try_vertex(i_dim) = comp_sum(i_dim) * a - &
                    simplex(i_dim,i_max) *b
            end do

            cost_try = f(try_vertex)

            ! if lower cost than max cost, replace the max vertex
            if ( cost_try < cost(i_max) ) then

                ! update the component_sum of the simplex
                do i_dim = 1, n_dim
                    comp_sum(i_dim) = comp_sum(i_dim) +&
                        try_vertex(i_dim) - simplex(i_dim,i_max)
                end do

                ! replace the max vetex with the trial vertex
                simplex(:,i_max) = try_vertex
            end if

        end function try_stretch

    end function fmin

    real function interp1d(table,x,x_scale,x_start)
        real, intent(in), dimension(:) :: table
        real, intent(in) :: x, x_scale, x_start
        real :: df, dx, f1, x1
        integer :: ix1, nx
        integer :: stat
        stat = 0

        nx = size(table)
        ix1 = floor((x-x_start)/x_scale) + 1
        dx = (x - x_start) - x_scale*(ix1-1)

        if (ix1 < 1) then
            ix1 = 1
            stat = 1
        else if (ix1 > nx) then
            ix1 = nx
            stat = 1
        else if (ix1 == nx) then
            if  (abs(dx) < tiny(dx)) then
                interp1d = table(nx)
                return
            else
                stat = 1
            end if
        end if

        f1 = table(ix1) 
        df = table(ix1+1) - f1
        interp1d = f1 + df / x_scale * dx

        if (stat == 1) then
            interp1d = table(ix1)
            write(*,*) "warning: interpolation exceeded table"
            write(*,*) "x: ", x
            return
        end if

    end function interp1d

    real function interp2d(table,x,x_scale,x_start, y,y_scale,y_start)
        real, intent(in), dimension(:,:) :: table
        real, intent(in) :: x, x_scale, x_start
        real, intent(in) :: y, y_scale, y_start
        real, dimension(size(table,1)) :: x_table
        real, dimension(2) :: y_table
        integer :: nx, ny, iy1, i
        real :: y1, dy

        nx = size(table,1)
        ny = size(table,2)
        iy1 = floor((y-y_start)/y_scale) + 1

        if (iy1 < 1) then
            iy1 =  1
        else if (iy1 >= ny) then
            iy1 =  ny - 1
        end if

        y1 = (iy1-1)*y_scale + y_start
        dy = y - y1 

        ! interpolate first y
        do i=1,nx
            x_table(i) = table(i,iy1)
        end do
        y_table(1) = interp1d(x_table,x,x_scale,x_start)

        ! interpolate second y
        do i=1,nx
            x_table(i) = table(i,iy1+1)
        end do
        y_table(2) = interp1d(x_table,x,x_scale,x_start)

        ! interpolate between points
        interp2d = interp1d(y_table,y,y_scale,y1)

    end function interp2d


end module math
