function bool = checkObstacle(node)
% checkObstacle checks if the given not is an obstacle or free node using
% half planes and semi algebraic expressions.
x = node(1);
y = node(2);

bool=0;

% Equation of Circle
fcircle = (x-180)^2+(y-120)^2-225;

% Equation of half planes of the polygon
f12 = (-41/25)*x+(1259/5)-y;
f23 = 14-y;
f34 = (37/20)*x-(1484/5) - y;
f45 = (-38/23)*x+(8317/23) - y;
f57 = (34/45)*x-(107/3) - y;
f56 = (38/7)*x-(5647/7) - y;
f67 = (-2/19)*x+(1285/19) - y;

if((x <= 0)||(y <= 0)||(x >= 250)||(y >= 150))
    bool=1;
    return
elseif((x>=55)&&(x<=105)&&(y<=112.5)&&(y>=67.5))
    bool=1;
    return
elseif(fcircle<=0)
    bool=1;
    return
elseif(((f12<=0)&&(f23<=0)&&(f34<=0)&&(f45>=0)&&(f57>=0)) && (~((f56<=0)&&(f67<=0))))
    bool=1;
    return
end
end

