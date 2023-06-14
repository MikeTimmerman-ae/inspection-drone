rng(100,"twister");

test_map = load("testing_map2.mat");
%whos -file testing_map.mat omap3D

Data = test_map.omap2D;
omap2 = test_map.omap2D;
% Consider unknown spaces to be unoccupied
omap2.FreeThreshold = omap2.OccupiedThreshold;
omap = occupancyMap(Data,1);
omap.FreeThreshold = Data.FreeThreshold;
omap.OccupiedThreshold = Data.OccupiedThreshold;

targets = [[1, 1];[300, 230];[700, 230];[700, 730];[300, 730];];
targ = [2,3,4,5];
targets2 = [[0, 0, 0];[300, 230, 0];[700, 230, 0];[700, 730, 0];[300, 730, 0];];
% disp(size(targets, 1));
%%%%%%%%%%%

subarrays = {};

% Loop through each array in the cell array
for k = 1:size(targets,1)

   
    % Get combinations of current length
    comb_k = nchoosek(targ, k);
    
   
    for j = 1:size(comb_k, 1)
        % disp(size(comb_k(j, :),2));
       
        subarrays{end + 1} = comb_k(j, :);
        
    end

end
% disp(subarrays);
array = [];
array2 = [];
hold on
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

for i = 1:size(subarrays, 2)
    % disp(size(subarrays{i},1));
    
    subarr = [1, 1];
    subarr2 = [0,0,0];
    for j = 1:size(subarrays{i}, 2)
        
        subarr = [subarr; targets(subarrays{i}(j), :)];
        subarr2 = [subarr2; targets2(subarrays{i}(j), :)];

        
    % states_tot = cat(1, states_tot, states(2:size(states, 1), :));
       
    end
    % disp(subarr2)
    sum_tot = 15;
    sum_tot2 = 15;
    for k = 2:size(subarr,1)
            
            [waypoints, sum_wp] = wp(subarr(k-1, :), subarr(k, :), omap);
            [waypoints2, sum_wp2] = wp2(subarr2(k-1, :), subarr2(k, :), omap2);
            plot(waypoints(:,1), waypoints(:,2), "b")
            plot(waypoints2(:,1), waypoints2(:,2), "r")
            sum_tot = sum_tot + sum_wp;
            sum_tot2 = sum_tot2 + sum_wp2;
    end
    percent_dff2 = (abs(sum_tot- sum_tot2)/((sum_tot + sum_tot2)/2))*100;
    % disp("--------")
    % disp(percent_diff2);
    % disp(sum_tot);
    % disp(sum_tot2);
    % disp("--------")
    array = [array; [percent_dff2, sum_tot, sum_tot2]];
    
    % array = cat(2, array, subarr);
    % array2 = cat(2, array2, subarr2);
end

hold off
disp(array);

M = mean(array(:,1));
disp(M);
