function birdseye(xs, ts);
    subplot(111);
    plot(xs(:,1), xs(:,2));
    xlabel('X-Position');
    ylabel('Y-Position');
    title('Birdseye View');
end
