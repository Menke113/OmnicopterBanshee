

function [u, error_history] = PID(state_desired, state_current, iteration, error_history, errorSum, kP, kI, kD,timestep)
    
    error = state_desired - state_current;
      
    error = error(4:6);

    error_history(:,iteration) = error;

    errorSum = errorSum + (error .* timestep);
            
    prop_term = kP .* error;

    int_term = kI .* errorSum;

    deriv_term = kD .* ((error - error_history(:,(iteration - 1)))./timestep);
    
    u = prop_term + int_term + deriv_term;
     

end
















