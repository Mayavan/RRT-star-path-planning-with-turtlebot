function [Xnear, position] = get_best_parent(Tree, neighbourhood)
min = Tree(neighbourhood(1)).costToCome;
Xnear = Tree(neighbourhood(1)).node;
position = neighbourhood(1);
for i = 2:size(neighbourhood, 2)
    if (min > Tree(neighbourhood(i)).costToCome)
        min = Tree(neighbourhood(i)).costToCome;
        Xnear = Tree(neighbourhood(i)).node;
        position = neighbourhood(i);
    end
end
end

