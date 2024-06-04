%% Tuning of the PI controller or any parameters using Particle Swarm Optimization 

%% Initialization

n = 10;           % Size of the swarm " no of birds "
bird_setp = 10;   % Maximum number of "birds steps"
dim = 2;          % Dimension of the problem (can be changed to 3 then we can optimize a PID controller) 



c2 =1.2;          % PSO parameter C1 
c1 = 1.2;         % PSO parameter C2 
w =0.9;           % pso momentum or inertia  
fitness=0*ones(n,bird_setp);
                                       %-----------------------------%
                                       %    initialize the parameter %
                                       %-----------------------------%
                                       
R1 = rand(n,dim);
R2 = rand(n,dim);
current_fitness =0*ones(n,1);

                                 %------------------------------------------------%
                                 % Initializing swarm and velocities and position %
                                 %------------------------------------------------%
                     
current_position = abs(10*(rand(n,dim)-.5));
velocity = .3*randn(n,dim) ;
local_best_position  = current_position;

                                 %-------------------------------------------%
                                 %     Evaluate initial population           %           
                                 %-------------------------------------------%

for i = 1:n
   % m=9
 %current_fitness =tracklsq(current_position(:,i));
 pid=current_position(i,:);
         ki = pid(1); %% This is an example!
         kd = pid(2); %% This is an example!
      
         sprintf('The value of interation Kp= %3.0f, Kd= %3.0f', pid(1),pid(2)); 
         % Compute function value
         simopt = simset('solver','ode45','SrcWorkspace','Current','DstWorkspace','Current');  % Initialize sim options
         [tout,xout,yout] = sim('Simulink model name',[0 1],simopt);
         % compute the error Simulink to Matlab Script
 
         
        sys_overshoot=max(V_Actual)-6350; %% V_Actual is an example and 6350 is the reference voltage 

         m=abs(e); %% (e) is an error with a sample time from the Simulink model 
         error=sum(m);
         F=error+sys_overshoot;
         current_fitness(i)=F;  
end
local_best_fitness  = current_fitness ;
[global_best_fitness,g] = min(local_best_fitness) ;

for i=1:n
    globl_best_position(i,:) = local_best_position(g,:) ;
end
globl_best_position;

                                               %-------------------%
                                               %  VELOCITY UPDATE  %
                                               %-------------------%

velocity = w *velocity + c1*(R1.*(local_best_position-current_position)) + c2*(R2.*(globl_best_position-current_position));

                                               %------------------%
                                               %   SWARMUPDATE    %
                                               %------------------%                                          
            
current_position = current_position + velocity ;

                                               %------------------------%
                                               %  evaluate a new swarm   %
                                               %------------------------%                                           
%% Main Loop
iter = 0 ;        % Iterations’counter
while  ( iter < bird_setp )
iter = iter + 1

for i = 1:n,
 pid=current_position(i,:);
         ki = pid(1);
         kd = pid(2);
  
         sprintf('The value of interation Kp= %3.0f, Kd= %3.0f', pid(1),pid(2)); 
         % Compute function value
         simopt = simset('solver','ode45','SrcWorkspace','Current','DstWorkspace','Current');  % Initialize sim options
         [tout,xout,yout] = sim('Simulink model name',[0 1],simopt);
         % compute the error 
                
         sys_overshoot=max(V_Actual)-6350;

         m=abs(e);
         error=sum(m)
         F=error+sys_overshoot;
         current_fitness(i)=F;  
end

for i = 1: n
        if current_fitness(i) < local_best_fitness(i)
           local_best_fitness(i)  = current_fitness(i);  
           local_best_position(i,:) = current_position(i,:);
        end   
 end

 [current_global_best_fitness,g] = min(local_best_fitness);
    
if current_global_best_fitness < global_best_fitness
   global_best_fitness = current_global_best_fitness;
   
    for i=1:n
        globl_best_position(i,:) = local_best_position(g,:);
    end
   
end

 velocity = w *velocity + c1*(R1.*(local_best_position-current_position)) + c2*(R2.*(globl_best_position-current_position));

 current_position = current_position + velocity; 
  
 sprintf('The value of interation iter %3.0f ', iter );


end %% end of while loop it means the end of all steps that the birds move it 
                      

            %xx=fitness(:,10);
            %[Y,I] = min(xx);
            %current_position(:,I)
            

         ki = globl_best_position(n,1);
         kd = globl_best_position(n,2);  
    
