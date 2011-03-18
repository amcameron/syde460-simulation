function [ps, ds] = intermediaries(xs, ts)
    ps = ones(size(ts,2), 2);
    ds = ones(size(ts,2), 2);
    for i = linspace(1,size(ts,2),size(ts,2))
	ps(i,1:2) = planpath(xs(i,:));
	_xs       = localize_state(xs(i,:)');
	ds(i,1:2) = controller(_xs, xs(i,13:14)');
    end
end
