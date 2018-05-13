function neighbourhood = get_neighbourhood(Tree, Xnew, region_radius)
neighbourhood = [];
for i = 1:size(Tree,2)
    if (norm(Tree(i).node - Xnew) < region_radius)
        neighbourhood = cat(2, neighbourhood, i);
    end
end
end

