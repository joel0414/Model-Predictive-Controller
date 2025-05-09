% Predicts outputs based on ode23 which is less accurate than ode45 but
% faster

% ts = sampling time
% x0 = inital state [0;0]
% u = vector of control u length of p
% p = prediction horizon

function y_out = predic_out(ts,x0,u,p)

    % Preallocate predicted output array of one column and p rows
    % This will store the outputs(position)
    % 1 row with p columns
    y_out = zeros(1,p);
    

    % Preallocate current state
    % array of two rows and p+1 row
    % need an extra column because we have a current x0
    %[0 1 ....] position
    %[0 1 ....] velocity

    % this stores all our states position and velocity
    x_curr = zeros(2, p+1);

    % We start by adding our initial state as our first state 
    x_curr(:,1) = x0;

    % Loop from 1 to p to get our predicted out values
    for i = 1 : p

        % Simulate with ode23 to get our sim_x
        [t , sim_x] = ode23(@(t,x) sys_S(t, x_curr(:,i) , u(i)),[0 ts], x_curr(:,i));

        % ode gives us multiple states from 0 to ts
        % we only care about the last one
        % original sim_x is given as a [x1 ,x2] but we need
        % [x1 ; x2] so the vector is transpose

        x_curr(:,i+1) = transpose(sim_x(end,:));
        
        % We only care about the first row as it stores x1
        y_out(i) = x_curr(1,i+1);

    end


end