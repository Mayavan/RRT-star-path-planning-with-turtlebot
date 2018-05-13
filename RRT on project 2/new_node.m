function Xnew = new_node(Xnear, Xrand, branch_length)

slope = (Xrand(2)-Xnear(2))/(Xrand(1)-Xnear(1));

adjuster = branch_length * sqrt(1/(1+(slope^2)));
possibleNew(1,1) = Xnear(1) + adjuster;
possibleNew(1,2) = Xnear(2) + slope * adjuster;

possibleNew(2,1) = Xnear(1) - adjuster;
possibleNew(2,2) = Xnear(2) - slope * adjuster;

distance1 = norm(Xrand - possibleNew(1,:));
distance2 = norm(Xrand - possibleNew(2,:));
if distance1 < distance2
    Xnew = possibleNew(1,:);
else
    Xnew = possibleNew(2,:);
end
end

