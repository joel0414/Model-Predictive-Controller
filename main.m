clc;
clear;

% === Seeting up time vector ===
t_i = 0;    % Intital time
t_f = 5;    % Final time
ts = 0.01;  % sampling time
t = 0 :ts: 5; % time vector


% ==== Parameters ====
p = 30;     % prediction horizon
m = 10;      % control horizon
x0 = [1;0];  % intial state
u_prev = 0; % Previous U


% === Final storage variables ====
x_store = zeros(2, length(t));
y_store = zeros(1, length(t));
u_store = zeros(1, length(t));
j_store = zeros(1, length(t));

% Store the intial state
x_store(:,1) = x0;
y_store(1) = x0(1);

% Vector of the possible u's we can try
u_val_size = 50;
u_val = linspace(-1, 1, u_val_size);  % Values of U from -1 to 1 [-1, ..., 1]
% Number of trials to go through for the optimizer
n_trials = 300;

% === Getting Reference Signal
ref = y_ref(t);

x = x0;
u = u_prev;
tic
% Simulating for every time step
% Since we simulate from x(k) to x(k+1)
% -1 is needed to not go over the index
for k = 1:length(t) - 1 

    % Check if looking ahead is possible
    % For example if length(ref) = 15 and p = 4
    % At k = 12  ===>   (k+p) = 12 + 4 = 16 => out of the index of ref
    % You get an "Index exceeds" error.
    if k + p <= length(ref)
        % Looking into the future p steps
        % Assume p = 15
        % [1 + 1 : 1 + 15] = [2 : 16] we get 2nd index to 16th of the ref
        % [2 + 1 : 2 + 15] = [3 : 17] we get 3rd index to 17th of the ref
        % We start at k+1 because we need to look into the future not at k
        ref_seg = ref(k+1 : k+p);
    else
        % We need to find how far we are to the end
        dis_end = length(ref) - k;

        % Get the most amount of values that we can
        possible_values = ref(k+1 : k + dis_end);

        % Get the last value possible
        last_value = possible_values(:, end);
        
        % We now need to repeat the last possible value p amount of times
        % We start by starting with possible values
        % we then use repmat(last value, 1 row, p - dis_end columns)
        % p - dis_end because it gets the remaining reference needed to be
        % seen
        % if p = 5 and dis_end = 2 => 5-2 = 3 we need 3 copies of the final
        % value
        % we truncate those two vectors into one

        ref_seg = [possible_values, repmat(last_value , 1, p - dis_end)];
    end

    % ==== Optimizer ====

    % Best cost must start with inf and not 0 or else nothing will beat
    % that
    best_cost = inf; 
    % The best vector of U's with the lowest cost
    best_u = [];

    % Lets loop for n number of trials
    for j = 1: n_trials

        % Get a vector of random u's of size 1 x m 
        rand_u = randsample(u_val , m, true);

        % Now we must copy the last random u copy it to make size 1xp

        % Get the last u from rand_u and copy it to get a vector of
        % size 1x p - m
        % p - m because we need to know how many of them we need left
        % If p = 10 and m = 5  ====> 10 - 5 = 5 remaining u's
        
        remaining_u = repmat(rand_u(:,end) , 1 , p - m);
        % [U1 , U2 , U3 , Ulast...... Ulast]
        trial_u = [rand_u, remaining_u];

        % Now we calculate the cost of these U's

        cost = cost_function(ts, x, trial_u, p, ref_seg, u_prev);
        
        % If the current cost is less than the best cost
        if cost < best_cost
            % Set this current cost as the best one
            best_cost = cost;
            % Set the current trial u as the best u
            best_u = trial_u;
        end
    end
    % ==== After Optimizing ===

    % We save the very best U
    % the very first U from the best_u vector
    u = best_u(1);
    u_store(k) = u;
    
    % Apply this U
    % We only care about the state
    [~,x_out] = ode45(@(t,x) sys_S(t,x,u),[0,ts],x);
    % We must get the last row than transpose it 
    x = transpose(x_out(end,:));

    % Store this state from the U
    x_store(:,k+1) = x;

    % Store the output from this u
    % Add Gaussian noise to output
    w = sqrt(0.001) * randn;
    y_store(k+1) = x(1) + w;

    % Store the best cost
    j_store(k) = best_cost;

    % Update previous U to the one we just used
    u_prev = u;
end
toc

% === Plot Results ===
figure;
plot(t, y_store, 'b', t, ref, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output y');
legend('y(t)', 'y_{ref}(t)');
title('System Output vs Reference');

figure;
plot(t, u_store, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Input u');
title('Control Input');
figure;

figure;
plot(t, j_store, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Cost J');
title('MPC Cost Function');