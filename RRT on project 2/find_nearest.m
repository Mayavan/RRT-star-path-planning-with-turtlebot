function Xnear = find_nearest(Xrand, Tree)
min_distance = 99999;
for ii =1:size(Tree, 2)
    distance = norm(Xrand - Tree(ii).node);
    if distance < min_distance
        min_distance = distance;
        Xnear = Tree(ii).node;
    end
end
end

