function bool = hasObstacle(Xnear, Xnew)
% To check if all points between are in the obstacle space or not

bool =0;

diff1 = (Xnew(1) - Xnear(1));
diff2 = (Xnew(2) - Xnear(2));

% take the greater difference
if abs(diff1) > abs(diff2)
    diff = diff1;
    decimated_index = 2;
else
    diff = diff2;
    decimated_index = 1;
end

% Creates set of points between two points
points_to_check=[Xnear];
for ii = 1:abs(diff)
point(1) = Xnear(1) + ii*diff1/abs(diff);
point(2) = Xnear(2) + ii*diff2/abs(diff);


point(decimated_index) = floor(point(decimated_index));
points_to_check = cat(1, points_to_check, point);

if floor(point(decimated_index)) ~= point(decimated_index)
    point(decimated_index) = point(decimated_index) + 1;
    points_to_check = cat(1, points_to_check, point);
end

end

% returns true if one of the point in between is an obstacle
for jj = 1:(size(points_to_check,1)-1)
    if(checkObstacle(points_to_check(jj,:)))
        bool = 1;
    end
end
    
end

