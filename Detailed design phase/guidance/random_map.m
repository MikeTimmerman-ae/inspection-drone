% Import the robotics system toolbox
% roboticsAddons

% Specify grid size
gridSize = 250;
rng(100,"twister");
% Specify the number of random blocks
numBlocks = 50;

% Create the occupancy grid
omap_verification2 = occupancyMap(gridSize,gridSize,10);

% Create random blocks in the occupancy grid
for i = 1:numBlocks
    % Random block position (x, y)
    x = randi([1, gridSize],1,1);
    y = randi([1, gridSize],1,1);

    % Random block size (width, height)
    width = randi([100, 150],1,1);
    height = randi([100, 150],1,1);

    % Set block in occupancy grid
    setOccupancy(omap_verification2, [x, y], ones(height, width));
end

% Display the map
figure
show(omap_verification2)
filePath = fullfile(pwd, "verification_map.mat" );
save(filePath, "omap_verification2")
