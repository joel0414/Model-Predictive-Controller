% ts: sampling time
% x0 :  initial state
% y_ref = reference output for the next p steps
% p: prediction horizon
% u = control input array that is generating the cost
% u_prev = the previus u that was actually inputted into the system

function j_k = cost_function(ts,x0,u, p , y_ref, u_prev)

    y_out = predic_out(ts, x0, u, p);

    % === Reference Tracking Error ===
    % In this case k = 0 

    ref_track_error = 0;
    % For loop is needed for summation
    for i = 1:p
        % Gets error from comparing pred with ref and squaring it
        e_i =(y_out(i) - y_ref(i))^2; 
        % Sum all of the e_i
        ref_track_error = ref_track_error + e_i;
    end

    % === Smoothness of Input

    % Get first control change cost
    % u(1) - u(0)
    % indexing starts at 1 not zero in matlab
    % must use u_prev but cant be in the loop
    first_delta = (u(1) - u_prev)^2;
    control_change_cost = first_delta;

    % Find remaining terms
    %u_{i} - u_{i-1}
    for i = 2:p
        delta_u = (u(i) - u(i-1))^2;
        control_change_cost = control_change_cost + delta_u;
    end

    j_k = .8 * ref_track_error + .2 * control_change_cost;

end