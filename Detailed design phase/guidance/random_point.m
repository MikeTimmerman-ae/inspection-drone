
rng(68, "twister");


gridSize = 250;
numTargets = 50;
% disp(size(targets, 1));


verification_map = load("verification_map.mat");


Data = verification_map.omap_verification2;

% Consider unknown spaces to be unoccupied

omap = occupancyMap(Data,1);
omap.FreeThreshold = Data.FreeThreshold;
omap.OccupiedThreshold = Data.OccupiedThreshold;

x = randi([1, gridSize],1,1);
y = randi([1, gridSize],1,1);
%%%%%%%%%%%
disp(x);
disp(y)




occupied = checkOccupancy(omap,[x y]);
if occupied == 1

    while checkOccupancy(omap,[x y]) ~= 1

        x = randi([1, gridSize],1,1);
        y = randi([1, gridSize],1,1);
    
    end 
end
targets = [[1, 1];[x, y];];

% disp(occupied);
% disp(omap)
% show(omap)


% Define start and end position

verification_tot = [];


for i = 2:size(targets,1)
   
    [verification, sum_wp] = wp(targets(i-1, :), targets(i, :), omap);
    
    
    verification_tot = cat(1, verification_tot, verification(2:size(verification, 1), :));
  
end

  
verification_tot_new = [];
for i = 1:size(verification_tot, 1)
    if mod(i, 10) == 0
        
        verification_tot_new = cat(1, verification_tot_new, verification_tot(i, :));
    end
end
verification_tot_new = [verification_tot_new; [x y]];
number_of_points = size(verification_tot_new, 1);

mode_vector = zeros(number_of_points, 1)+2;
% mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;

verification_struct = struct('mode',[],'position',[], 'params',[]);
verification_struct = repmat(verification_struct,number_of_points, 1);

verification_struct(1).mode = uint8(1) ;
verification_struct(1).position = single([0;0;-10]);
verification_struct(1).params = single([0;0;0;0]);
verification_struct(number_of_points).mode = uint8(7) ;
verification_struct(number_of_points).position = single([verification_tot_new(number_of_points,1);verification_tot_new(number_of_points,2);-15]);
verification_struct(number_of_points).params = single([-1;-1;-1;-1]);



for i = 2:number_of_points-1
    verification_struct(i).mode = uint8(mode_vector(i-1,1)) ;
    verification_struct(i).position = single([verification_tot_new(i-1,1);verification_tot_new(i-1,2);-15]);
    verification_struct(i).params = single([0;0;0;0]);
end

assignin('base',"verification_struct", verification_struct);

% Plot the waypoints
z_zeros = zeros(size(verification_tot_new(:,1)));
z_zeros = z_zeros(:) +15;
hold on
plot3(verification_tot_new(:,1),verification_tot_new(:,2), z_zeros, "r")
% show(omap)
% scatter(targets(1,1),targets(1,2),30,".r")
% scatter(targets(size(targets, 1),1),targets(size(targets, 1),2),30,".g")
% plot(states_tot(:,1),states_tot(:,2), "-y")

% view([-31 63])
test_ = out.PoseOut;
plot3(test_(:, 1), test_(:, 2), -test_(:, 3), "b");
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
% axis([-100 100 -100 100])
legend("Generated Path", "Sim path")
grid on
hold off