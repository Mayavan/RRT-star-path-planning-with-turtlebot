function point = get_random_point(x,y)
% returns a random co-ordinate in the X x Y image
random = randi(x*y);
point = [round(random/150), mod(random,150)];
end

