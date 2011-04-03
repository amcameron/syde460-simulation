function [ps, ds] = intermediaries(xs, ts)
    ps = ones(size(ts,2), 2);
    ds = ones(size(ts,2), 2);
    for i = 1:size(ts,2)
	ps(i,1:2) = planpath(xs(i,:)');
	xs_       = localize_state(xs(i,:)');
	ds(i,1:2) = controller(xs_, xs(i,13:14)');
    end
end
