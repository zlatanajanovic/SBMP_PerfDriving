function leaf_val = leaf(length, ind)
    leaf_val = false;
    if (right(ind)>=length) && (left(ind)>=length)
        leaf_val = true;
    end
end