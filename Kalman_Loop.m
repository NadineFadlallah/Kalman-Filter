%% System Parameters
% All units are standard units System Dynamics Modeling (Ideal Model) 

m = 1;     % robot mass 
b = 0.5;   % Fricrtion coeffecient 
dt = 0.05; % Sampling time 
F = [1 0 dt 0;0 1 0 dt;0 0 1-(b*dt/m) 0;0 0 0 1-(b*dt/m)]; % What does this term: 1-(b*dt/m) mean ?! Ans:What term?! I see no terms! :"D
B = [0 0;0 0;dt 0;0 dt];                                   %Air Friction(drag) is subtractive of velocity which means deceleration and for relatively small velocities is linearly dependant on velocity (Laminar flow): F = -bv
H = [1 0 0 0;0 1 0 0];                                     %Calling from Newton's 2nd law: F = ma = -bv ==> a = -bv/m, and from laws of motion: v(i+1) = v(i) + at ==> v(i+1) = v(i) [1 - b*t/m]
% Noise Covariance Matrices (To be modified)                
% Your Parameter Tunning Starts Here 
%Q = zeros(4);      % Model Uncertainity 
load('u_motion.mat')
X = zeros(4,1000);
for index = 1:1000
X(:,index) = motion_model(0.05, u_motion);
end

Q = eye(4) .* (transpose(var(X.',1)) * ones(1,4)); % Model Uncertainity 

%R =eye(2)*(3) %zeros(2);      % Sensor Noise
z = zeros(2,1000);
for index = 1:1000
z(:,index) = measurement_model();
end
R = eye(2) .* (transpose(var(z.',1)) * ones(1,2));     % Sensor Noise 
% Your Parameter Tunning Ends Here
%% System Initializations
% State and Covariance matrix initialization Your initialization Starts Here 

X_init = zeros(4,1); % (You may try another initial state)
% The given ground truth assumes that X_init is zeros(4,1)
%P_init = eye(4);  % (To be Modified)
%P_init = [0.6 0 0 0; 0 0.6 0 0; 0 0 0.4 0; 0 0 0 0.4];
P_init = [0.04 0 0 0; 0 0.05 0 0; 0 0 0.04 0; 0 0 0 0.04]
% Your initialization Ends Here 

% X_est is an array for state values in all samples of the experiment 
N = 1000; % 1000 samples = 50 sec 
X_est = zeros(4,N);
X_est(:,1)=X_init;
% Control action sequence u is the sequence applied to the robot within
% specified time for the experiment 
u = generate_sequence(dt,N-1); % Why N-1 ? Ans: 'Cause why not? right? :"D
                               % Well, I guess the intial state is considered
                               % independant of the specified control action
%% Kalman Filter
% X_pre and P_pre are state vector and covariance matrix before correction X_post 
% and P_post are the values after Kalman correction 

X_pre = X_init;
X_post = X_init;
P_pre = P_init;
P_post = P_init;
Z_all = zeros(2,N-1);

for i=1:N-1
    % Your Code Starts Here 
    % 1. State Prediction step 
    X_pre = F*X_post + B*u(:,i);
    P_pre = F*P_post*F.' + Q;
    
    % 2. Kalman Gain calculation 
    S = H*P_pre*H.'+ R;
    K = (P_pre*H.')/S;
    
    % 3. State Correction step 
    Z = real_system(X_init, dt, u(:,i));
    Z_all(:,i) = Z;
    X_post = X_pre + K*(Z - H*X_pre);
    P_post = P_pre - K*H*P_pre;
    X_est(:,i+1)=X_post; 
end

%% Data Visualization
% Ground Truth 

close all 
load ground_truth 
plot(ground_truth(1,:),ground_truth(2,:));

% Your Result 
hold on 
plot(X_est(1,:),X_est(2,:),'r');
%hold on
%plot(Z_all(1,:),Z_all(2,:),'g');