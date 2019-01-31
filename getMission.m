function [waypointX, waypointY] = getMission_fn(X, Y)

            width = 0.5;
		    length = 0.5;
			swath = 0.05;
			pathShort = swath;
			pathLong = length;
            passes = width/(pathShort); % number of required passes to cover area
			number = passes*2;          % number of waypoints to cover area (2D) // round up or down?? future check this
            waypointX = zeros((number+1),1);
            waypointY = zeros((number+1),1);
			waypointX(1) = X + (pathLong/2); 
			waypointY(1) = Y;

			for  count = 2:number %loop to generate waypoints
				if mod(count,2) == 0 %EVEN
						waypointX(count)= waypointX(count-1);
						waypointY(count) = waypointY(count-1) + pathShort;
			
                else %ODD
					if(mod((count-3),4)  == 0)
						waypointX(count) = waypointX(count-1) - pathLong;
						waypointY(count) = waypointY(count-1) ;
				
                    else
						waypointX(count) = waypointX(count-1)+ pathLong;
						waypointY(count) = waypointY(count-1) ;
                    end
          
                end
            end
			waypointX(count+1) = X;
			waypointY(count+1) = Y;		
			
end