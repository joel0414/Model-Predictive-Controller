% Function to build our reference signal based on t

function y_r = y_ref(t)

    % Preallocating space for y_r depending on how long t is 
    y_r = zeros(1,length(t));
    
    for i = 1:length(t)
    
        % If 0 < t <1
        if 0 < t(i) && t(i) <= 1
            y_r(i) = .2;
        
        % If 1 < t <= 2
        elseif 1 < t(i) && t(i) <= 2
            y_r(i) = .1*(1+t(i).^2);
        
        % If t > 2
        elseif 2 < t(i)
            y_r(i) = .5;
            
        end
    end

end
