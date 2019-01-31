function difference = getAngle_fn(x1, y1, currentAngle, x2, y2)
x1 = x1;
x2 = x2;
currentAngle = currentAngle;
y1 = y1;
y2 = y2;

PI = 3.14159;
hd = atan2(double(y2-y1),double(x2-x1));
        if (x2-x1 >= 0 & y2-y1 >= 0) %// quadrant 1 (checked!)
            if (currentAngle > (hd)) 
               theta = (-((currentAngle) - hd));
            elseif(currentAngle <= (hd - PI))
               theta = (-((PI-abs(currentAngle)) + (PI - hd)));
            else
               theta = (hd - currentAngle);
        
            end
        end
        
       if (x2-x1 < 0 && y2-y1 >= 0) %//quadrant 2 (checked!
 
            if (currentAngle > (hd))
                theta = (-((currentAngle) - hd));
            elseif (currentAngle < hd & currentAngle > (hd - PI))
                theta = (hd - currentAngle);
            else
                theta = -((PI-hd) + (PI-abs(currentAngle)));
            end
       end
       
       if (x2-x1 < 0 && y2-y1 < 0) %//quadrant 3 (checked!)
 
            if ((currentAngle >= 0 ))
                if(currentAngle>= hd+PI)
                    theta = ((PI-abs(hd)) + (PI-abs(currentAngle)));
                else
                    theta = (-(abs(hd) + currentAngle));
                end
                
            else
                if (currentAngle <= hd)
                    theta = (abs(currentAngle)-abs(hd));
                else
                    theta = -(abs(hd)-abs(currentAngle));
                end
           
           end
       end
            
   
       
       if(x2-x1 > 0 && y2-y1 < 0) %//quadrant 4
 
            if ((currentAngle >= 0 ))
                if(currentAngle>= hd+PI)
                    theta = ((PI-currentAngle) + (PI-abs(hd)));
                else
                    theta = -(currentAngle + abs(hd));
                end
            else
                if (currentAngle <= hd)
                     theta =(abs(currentAngle)-abs(hd));
                else
                     theta = -(abs(hd)-abs(currentAngle));
                end
                
            end
       end
       difference = theta;
end

