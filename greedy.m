function [match_object, index_object] = greedy(cost_table, num_n, num_m, treshold)
	% allocate memory
	match_object = ones(num_n)*(treshold + 1);
	index_cost = ones(num_n)*(-1);
    index_object = ones(num_m)*(-1);

    % for the minimum in each origin object
    for n = 1:num_n
        for m = 1:num_m
            if match_object(n) > cost_table(n,m) && cost_table(n,m) < treshold
            	% pick lowest cost in a row
                match_object(n) = cost_table(n,m);
                index_cost(n) = m;
            end
        end
    end

    % check which of the columns is the lowest value
    for m = 1:num_m
        index = find(index_cost == m);
        for p = 1:(length(index)-1)
        	if match_object(index(p)) < match_object(index(p+1))
        		match_object(index(p+1)) = match_object(index(p));
        		index_object(m) = index(p+1);
        	else
        		index_object(m) = index(p);
        	end  
        end
    end
end