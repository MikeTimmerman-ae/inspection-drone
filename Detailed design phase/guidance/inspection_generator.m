function inspectionStatesSpaced = inspection_generator(angle, nh, bl)
    inspectionStates = [];
    % inspectionStates(1,:) = [0;0;0;0];
    inspectionStates(1,:) = [0;0;-nh;0];
    inspectionStates(2,:) = [sin(angle)*1.15* bl ; 0; -nh + cos(angle)*1.15* bl;0];
    inspectionStates(3,:) = [sin(angle)*1.15*bl; 6; -nh + cos(angle)*1.15*bl; 0];
    % inspectionStates(4,:) = [0.15*bl; 8; -nh + cos(angle)*0.15* bl; 0];
    % inspectionStates(5,:) = [0.15*bl; 8; -nh - 0.15*bl; 0];
    inspectionStates(4,:) = [0; 6; -nh; 0];
    inspectionStates(5,:) = [0; 6; -nh - 1.15*bl; 0];
    inspectionStates(6,:) = [0; 0; -nh - 1.15*bl; 0];
    inspectionStates(7,:) = [0; 0; -nh; 0];
    inspectionStates(8,:) = [-sin(angle)*1.15*bl; 0; -nh + cos(angle)*1.15*bl; 0];
    inspectionStates(9,:) = [-sin(angle)*1.15*bl; 8; -nh + cos(angle)*1.15*bl; 0];
    inspectionStates(10,:) = [-0.15*bl; 8; -nh  + cos(angle)*0.15*bl;0];
    inspectionStatesSpaced = [];
    
    inspectionStatesSpaced(1 ,:) = inspectionStates(1,:);
    k = 2;
    for i = 2:10
        % n = int((norm(inspectionStates(i, (1:3)) -  inspectionStates(i-1, (1:3)))) /10);
        if norm(inspectionStates(i, (1:3)) -  inspectionStates(i-1, (1:3))) > 80
            n = 8;
        elseif norm(inspectionStates(i, (1:3)) -  inspectionStates(i-1, (1:3))) > 50
            n = 5;
        elseif norm(inspectionStates(i, (1:3)) -  inspectionStates(i-1, (1:3))) > 30
            n = 3;
        else
            n = 2;
        end
        x = linspace(inspectionStates(i-1, 1), inspectionStates(i, 1), n);
        y = linspace(inspectionStates(i-1, 2), inspectionStates(i, 2), n);
        z = linspace(inspectionStates(i-1, 3), inspectionStates(i, 3), n);
        
        for j = 1:n-1
            inspectionStatesSpaced(k,:) = [x(j+1); y(j+1); z(j+1); 0];
            % disp(k)
            % disp(((i-2)*(n)) + j + 1);
            k = k + 1;
            
            
        end
        
    end
    % % disp(inspectionStatesSpaced);
end

