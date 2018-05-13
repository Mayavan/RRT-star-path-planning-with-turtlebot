function location = findParent(Tree, position_of_child)
    for i = 1:size(Tree, 2)
        x = ismember(position_of_child, Tree(i).branches);
        if(sum(x) > 0)
            location = i;
            return
        end
    end
end

