


function wpstruct = path_constructor(array)
    N = 10;
    wpstruct = struct("mode",[], "position", [], "params", []);
    wpstruct = repmat(wpstruct, (length(array)-1)*N, 1);
    prev = [0,0,0];
    for i = 1:length(array)
        wp = array{i};
       
        x1 = prev(1);
        y1 = prev(2);
        z1 = prev(3);
        x2 = wp{2}(1);
        y2 = wp{2}(2);
        z2 = wp{2}(3);
        x=linspace(x1,x2,N);
        y=linspace(y1,y2,N);
        z=linspace(z1,z2,N);
        len = length(x);
        % if i == 1
        %     len = length(x);
        % 
        % elseif i == length(array)
        % 
        %     len = length(x);
        % else
        % 
        %     len = length(x)-1;
        % end
        
        % len = length(x);
        
        if wp{1} == 1
            wpstruct(1).mode = uint8(wp{1});
            wpstruct(1).position = single([x2; y2; z2]);
            wpstruct(1).params = single(wp{3});
           
        else
            for j = 1:len
                display(j-i+1 + ((i-1)*len) -(N-2))
                
                
                    
                if j ~= length(x)
                    wpstruct(j-i+1 + ((i-1)*len) -(N-2)).mode = uint8(2);
                else
                    wpstruct(j-i+1 + ((i-1)*len)-(N-2)).mode = uint8(4);
                end
                wpstruct(j-i+1 + ((i-1)*len) -(N-2)).position = single([x(j); y(j); z(j)]);
                wpstruct(j-i+1 + ((i-1)*len) -(N-2)).params = single(wp{3});
            end
            
        end
        prev = [x2, y2, z2];


    end
    assignin('base','testStruct',wpstruct)
            
    %     wpstruct = struct("mode",[], "position", [], "params", []);
    % wpstruct = struct("mode",[], "position", [], "params", []);
    % wpstruct = repmat(wpstruct, 5, 1);
    % names = {"mode", "position", "params"};
    % takeoff = {uint8(mode) single(pos) single(params)};
    % args = [names;takeoff];
    % wpstruct = struct(args{:});
    % values = {uint8(2) single([x;y;z]) single([0;0;0;0])};
    % 
    % 
    % 
    % args2=[names;values];
    % wpstruct(end + 1) = struct(args2{:});
    % wpstruct = struct("mode",[], "position", [], "params", []);
    % 
    % 
    % x=linspace(x1,x2,N);
    % y=linspace(y1,y2,N);
    % z=linspace(z1,z2,N);
    % 
    % 
    % 
    % wpstruct(1).position = single([0;0;z]);
    % wpstruct(1).params = single([0;0;0;0]);
    % 
    % 
    % for i = 2:5
    % 
    %     wpstruct(i).position = single([x+i;y+i;z]);
    %     wpstruct(i).params = single([0;0;0;0]);
    % assignin('base','fcnStatus',wpstruct)
end


