%% Omnicopter Simulation 
%Close and clear all
close all; 
clear all; 
clc 

%% Parameters 
%Omnicopter Parameters
m = 1500; %grams 
I = [1, 0, 0;
    0, 0.87, 0.5; 
    0, -0.5, 0.87]; %moment of inertia tensor of the vehicle kg*m^2 - check this with solidworks model
l = 0.62/2; %m - length of the lever arm for props 1-4

kT  = 0.5; %Thrust Coefficients 
T_max = 1600; %maximum prptimpeller thrust in grams


%Physical Constants 
g = 9.81; %m/s^2

% Simulation Parameters
timestep = 0.01; %seconds
simulation_time = 10; %seconds
maximum_acceleration = 3; %g's 
maximum_iterations = simulation_time/timestep;

%Intial State [x, y, z, x_dot, y_dot, z_dot, q1, q2, q3, q4] - set to
%initially be upright 
x_0 = [0; 0; 0; 2.2; 2.2; 2.2; 1; 0; 0; 0];

%PID Gains
% kP = 1;
% kD = 0.5e-4; 
% kI = 300;

% kP = 100;
% kD = 0.5e-4; 
% kI = 300;

kP = 50;
kD = 30; 
kI = 20000;

integral_reset_timer = 1;

%Desired state - eventually make it change over time
state_desired = [0; 0; 0; 5.2; 5.2; 2.2; 1; 0; 0; 0];

%State history arrays 
global_state_history(:,1) = x_0;

%Initializations for the PID loop
u = zeros(1,3);

error_history = zeros(3,maximum_iterations-1);
error_history(:,1) = (x_0(4:6)-state_desired(4:6));
errorSum = 0;

thrust_history = zeros(8,maximum_iterations-1);
state_history = zeros(10,maximum_iterations-1);
state_history(:,1) = x_0;
time_history = 0:timestep:simulation_time-timestep;


%Initial Parameters
T = zeros(1,8);
omega = [0; 0; 0];
omega_dot = [0; 0; 0];


%Adjust relevant variables
m = m/1e3; 
T_max = T_max * 0.00980665;
maximum_acceleration = maximum_acceleration * g;
current_time = 0;
x = x_0;


%% Setup for optimization

    k_T1 = 0.005; % coeff of thrust for 5" props (thrust_motor = k_T * omega), these are just guesses right now and are probably very wrong
    k_T2 = 0.007; % coeff of thrust for 7" props (thrust_motor = k_T * omega), these are just guesses right now and are probably very wrong

    I1 = 0.01; % moment of inertia for 5" props (moment_motor = I * alpha)
    I2 = 0.015; % moment of inertia for 7" props (moment_motor = I * alpha)

    Ke = 100; % efficiency coeff, used in cost function to decide whether efficiency or responsiveness is prioritized
    Kr = 1; % responsiveness coeff, used in cost function to decide whether efficiency or responsiveness is prioritized

    w_curr = [0 0 0 0 0 0 0 0];

    w_history = zeros(8,100000);


%% PID Control Simulation
for iteration=2:maximum_iterations
    
    %Create u vector

    %TODO - Swap to individual motor control 
    T(1:4) = T_max.*u(3);
    T(5) = T_max * (1/sqrt(2)*(u(1)+u(2)));
    T(6) = T_max * (1/sqrt(2)*(-u(1)-u(2)));
    T(7) = T_max * (1/sqrt(2)*(-u(1)+u(2)));
    T(8) = T_max * (1/sqrt(2)*(-u(1)+u(2)));
    
%     Make sure thrusters don't go over maximum thrust 
    for i=1:length(T)
        if abs(T(i)) > T_max
            T(i) = T_max*sign(T(i));
        end
    end

    thrust_history(:,iteration) = T;

    %Calculate net thrust vectors in the body frame 
    T_body = [(T(5)-T(6)-T(7)+T(8))/sqrt(2);
        (T(5)-T(6)-T(7)+T(8))/(sqrt(2));
       sum(T(1:4))
    ];
    

    
    %Create "E" and "G" Matrices from quaternions for use of body dynamics 
    q = x(7:10);
  
    E = [-q(2) q(1) -q(4) q(3);
        -q(3) q(4) q(1) -q(2);
        -q(4) -q(3) q(2) q(1)
    ];

    G = [-q(2) q(1) q(4) -q(3);
        -q(3) -q(4) q(1) q(2);
        -q(4) q(3) -q(2) q(1)
    ];
    
    %Quaternion Rotation Matrix 
    R = E*G';   
    
    %Calculate current angular velocity
    omega = 2*G*q;

    %Calculate net moments in the body frame - TODO add contributions from
    %propellers 
    M_body = [T(2)*l-T(4)*l;
        T(1)*l-T(3)*l;
        0
    ];

     [T_optim, M_optim, w_new] = Optim(T_body, M_body, w_curr, k_T1, k_T2, Ke, Kr, l); % call optimization function with T_body and M_body as desired net thrust and moment
     % rather than assuming that these are attainable

%     T_body
%
%     M_body
% 
%     T_optim
% 
%     M_optim


    %% Account for propellers not perfectly reaching commanded angular rates using Gaussian noise

    for j = 1:8

        w_new(j) = w_new(j) * (1 + randn./100);

    end

     %% Calculate the moments from the motors 
     % (will mess up the sim if this is actually taken into account in the net moment, due to it not being accounted for in the optimization)

    alpha = (w_new - w_curr)./timestep;

    total_motor_moments(1:4) = alpha(1:4).*I1;

    total_motor_moments(5:8) = alpha(5:8).*I2;

    motor_moments_body = [sum(total_motor_moments(5:8))/sqrt(2); sum(total_motor_moments(5:8))/sqrt(2); sum(total_motor_moments(1:4))];

%      M_body = M_optim + motor_moments_body

    %% Set M_body and T_body to the thrusts created by the omega vector found from the optimization

    T_body = T_optim; % set the net thrust to the net thrust given by the optimization

    M_body = M_optim; % set the net moment to the net moment given by the optimization

    %%

    w_history(:, iteration) = w_new';  % save the omega vector history


    %Angular rate derivative in the body frame 
    omega_dot = I^-1 * M_body - I^-1 * (cross(omega,I*omega));

    %Map to quaternion rates in the inertial frame 
    q_dot = 1/2*G'*omega_dot;
    
    
    %Force in the inertial frame
    F_net = R*T_body;
    
    
    %Calculate Nonlinear State Derivative 
    dot_x = @(t, x)([x(4); ...
        x(5);...
        x(6);...
        F_net(1)/m;...
        F_net(2)/m;...
        F_net(3)/m - g; ...
        q_dot(1); ...
        q_dot(2); ...
        q_dot(3); ...
        q_dot(4)]);
        
    %Perform RK4 On State Derivative
    next_state = RK4NumInt(current_time, x,  timestep, dot_x);

    %% Account for sensor noise by applying a Gaussian noise distribution to the state vector

    for j2 = 1:6

        next_state(j2) = next_state(j2) * (1 + randn./100);

    end

    %Append current state to the state history matrix 
    state_history(:,iteration) = next_state;

    %Calculate next time 
    current_time = current_time + timestep;

    w_curr = w_new;

    
    %% PID loop 
    %Reset integral error history
    if(iteration == 2)
        errorResetTimer = 0;
    
    else
        errorResetTimer = errorResetTimer + timestep;
    end
    
    if(errorResetTimer > integral_reset_timer)
        errorSum = zeros(1,3);
    end

    %Compute PID Control 
    [u, error_history] = PID(state_desired, next_state, iteration, error_history, errorSum, kP, kI, kD,timestep);
    
    %Set current state to be the next state
    x = next_state;
end


%% Plotting of States 
%X Position Subplots  
figure
subplot(3,1,1)
plot(time_history,state_history(1,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('X Position (m)')
grid on;
hold on 
% plot(time_history, ones(maximum_iterations-1,1)*state_desired(1),'LineStyle','--','LineWidth',2)

subplot(3,1,2)
plot(time_history,state_history(4,:),'LineWidth',2)
hold on 
plot(time_history, ones(length(time_history),1)*state_desired(4),'LineStyle','--','LineWidth',2)

xlabel("Time (s)")
ylabel('X Velocity (m/s)')
grid on;


subplot(3,1,3)
plot(time_history,thrust_history(5:8,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Thrust (N)')
legend('$T_5$', '$T_6$','$T_7$','$T_8$','interpreter','latex')
grid on;

%Y Position Subplots  
figure
subplot(3,1,1)
plot(time_history,state_history(2,:),'LineWidth',2)
hold on 
% plot(time_history, ones(maximum_iterations-1,1)*state_desired(2),'LineStyle','--','LineWidth',2)
xlabel("Time (s)")
ylabel('Y Position (m)')
grid on;

subplot(3,1,2)
plot(time_history,state_history(5,:),'LineWidth',2)
hold on 
plot(time_history, ones(length(time_history),1)*state_desired(5),'LineStyle','--','LineWidth',2)
xlabel("Time (s)")
ylabel('Y Velocity (m/s)')
grid on;


subplot(3,1,3)
plot(time_history,thrust_history(5:8,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Thrust (N)')
legend('$T_5$', '$T_6$','$T_7$','$T_8$','interpreter','latex')
grid on;

%Z Position Subplots  
figure
subplot(3,1,1)
plot(time_history,state_history(3,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Z Position (m)')
grid on;
hold on 
% plot(time_history, ones(maximum_iterations-1,1)*state_desired(3),'LineStyle','--','LineWidth',2)

subplot(3,1,2)
plot(time_history,state_history(6,:),'LineWidth',2)
hold on 
plot(time_history, ones(length(time_history),1)*state_desired(6),'LineStyle','--','LineWidth',2)
xlabel("Time (s)")
ylabel('Z Velocity (m/s)')
grid on;


subplot(3,1,3)
plot(time_history,thrust_history(1:4,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Thrust (N)')
legend('$T_1$', '$T_2$','$T_3$','$T_4$','interpreter','latex')

grid on;


%Error Plots 
figure 
subplot(3,1,1)
plot(time_history,error_history(1,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('X Error (m)')
grid on;

subplot(3,1,2)
plot(time_history,error_history(2,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Y Error (m)')
grid on;


subplot(3,1,3)
plot(time_history,error_history(3,:),'LineWidth',2)
xlabel("Time (s)")
ylabel('Z Error (m)')
grid on;


%3D Position Plot
figure
scatter3(state_history(1,:),state_history(2,:),state_history(3,:),[],time_history,'filled')
colormap(bone)
cbar = colorbar;
xlabel('X Position(m)')
ylabel('Y Position(m)')
zlabel('Z Position(m)')
ylabel(cbar,'Time (s)')

%Save data for animation
save('10_DOF_constrained_data','time_history','state_history','error_history','thrust_history')



%Runge-Kutta 4 Function
function [next_state] = RK4NumInt(t_0, x_0, dt, f)
    k0 = f(t_0,x_0);
    k1 = f(t_0 + dt/2, x_0+dt*k0/2);
    k2 = f(t_0 + dt/2, x_0+dt*k1/2);
    k3 = f(t_0 + dt, x_0+dt*k2);

    next_state = x_0 + 1/6*(k0 + 2*k1 + 2*k2 + k3)*dt;
end 

%% Optim: constrained optimization using fmincon that finds an 8D motor thrust vector, optimizing for responsiveness or efficiency

function [T_new, M_new, w_new] = Optim(T_net_body, M_net_body, w_curr, k_T1, k_T2, Ke, Kr, moment_arm)

    cost_fxn = @(w_new) (Ke * norm(w_new.^2) + Kr * norm((w_new.^2 - w_curr.^2)));

%       efficiency term  ^^^^^^^^^^^^^^^
%                      responsiveness term  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Tmotors_body = [0 0 k_T1; 0 0 k_T1; 0 0 k_T1; 0 0 k_T1; 
    (k_T2)/sqrt(2) (k_T2)/sqrt(2) 0; -(k_T2)/sqrt(2) (k_T2)/sqrt(2) 0; 
    -(k_T2)/sqrt(2) -(k_T2)/sqrt(2) 0; -(k_T2)/sqrt(2) (k_T2)/sqrt(2) 0]; % coefficients to convert from omegas to thrusts (omega^2 * k_T = thrust)

    torque_coeff = k_T1 * moment_arm;

    Mmotors_body = [0 torque_coeff 0; torque_coeff 0 0; 0 -torque_coeff 0; -torque_coeff 0 0; 
    0 0 0; 0 0 0; 0 0 0; 0 0 0]; % coefficients to convert from omegas to torques (omega^2 * k_T * moment_arm = torque)

    Aeq = [Tmotors_body Mmotors_body];
    
    beq(1:3,1) = T_net_body; % T_net_body is desired net thrust force in body frame
    
    beq(4:6,1) = M_net_body;  % M_net_body is desired net moment in body frame

    A = [k_T1 0 0 0 0 0 0 0;
        0 k_T1 0 0 0 0 0 0;
        0 0 k_T1 0 0 0 0 0;
        0 0 0 k_T1 0 0 0 0;
        0 0 0 0 k_T2 0 0 0;
        0 0 0 0 0 k_T2 0 0;
        0 0 0 0 0 0 k_T2 0;
        0 0 0 0 0 0 0 k_T2]; % coefficients for linear inequality

    b = [1600 1600 1600 1600 2200 2200 2200 2200]; % these are the max thrusts that each motor can produce

    b = b./102; % convert from grams to Newtons

    % fmincon will have to make sure that each motor creates a total thrust
    % no greater than these values

    % I currently am not constraining it with the max moments that each
    % motor can produce, but that should be added in  the future when the
    % moments are taken into account
    
    w_new_squared = fmincon(cost_fxn, w_curr, A, b, Aeq', beq); % w is omega, this is the omega vector with each angular rate squared

    w_new = (abs(w_new_squared)).^(0.5); % new omega vector that should result in the correct net thrust and moment being created


    for i = 1:8 % fix the signs that were lost in taking the absolute value of w_new_squared
        
        if(w_new_squared(i)<0)

            w_new(i) = -w_new(i);

        end

    end

    T_motors_new(1:4) = w_new_squared(1:4) .* k_T1;

    T_motors_new(5:8) = w_new_squared(5:8) .* k_T2;

    T_motors_new;

    T_new = [(T_motors_new(5)-T_motors_new(6)-T_motors_new(7)+T_motors_new(8))/sqrt(2);
        (T_motors_new(5)-T_motors_new(6)-T_motors_new(7)+T_motors_new(8))/(sqrt(2));
       sum(T_motors_new(1:4))
    ];

    M_new = [T_motors_new(2)*moment_arm-T_motors_new(4)*moment_arm;
        T_motors_new(1)*moment_arm-T_motors_new(3)*moment_arm;
        0
    ];

    % if it works, then T_new should be equal to T_net_body and M_new
    % should be equal to M_net_body

end

