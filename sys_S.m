% This functions defines our system S which was given to us

function dxdt = sys_S(t,x,u)
    x_1 =  x(1);
    x_2 = x(2);

    dx_1 = x_2;
    dx_2 = -.01 * x_2 + u;

    dxdt = zeros(2,1); % [0;0]  2 row vector

    dxdt(1) = dx_1;
    dxdt(2) = dx_2;
    
end