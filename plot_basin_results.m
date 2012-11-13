function plot_basin_results (filename)
    % Failure types
    no_failure = 0;
    fallen_backwards = 1;
    fallen_forwards = 2;

    load (filename);
    [I_fwd, J_fwd] = find (FAILURE_MODE == (fallen_forwards  * ones(size(FAILURE_MODE))) );
    [I_bwd, J_bwd] = find (FAILURE_MODE == (fallen_backwards * ones(size(FAILURE_MODE))) );

    figure();
    title ('Number of Bounces Completed');
    xlabel ('l-dot_0');
    ylabel ('\phi-dot_0');

    hold on;
    pcolor (L_DOT_0S, PHI_DOT_0S, NUM_BOUNCES_COMPLETED);
    plot (L_DOT_0S(1, J_fwd), PHI_DOT_0S(I_fwd, 1), 'wx', 'MarkerSize', 5, 'MarkerFaceColor', 'white');
    plot (L_DOT_0S(1, J_bwd), PHI_DOT_0S(I_bwd, 1), 'wo', 'MarkerSize', 5, 'MarkerFaceColor', 'white');
    shading interp;
    colorbar;
    hold off;
end