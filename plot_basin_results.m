function plot_basin_results (filename)
    % Failure types
    no_failure = 0;
    fallen_backwards = 1;
    fallen_forwards = 2;

    load (filename);
    for b=1:size(PHI_0S, 3)
        [I_fwd, J_fwd] = find (FAILURE_MODE(:,:,b) == (fallen_forwards  * ones(size(FAILURE_MODE(:,:,b)))) );
        [I_bwd, J_bwd] = find (FAILURE_MODE(:,:,b) == (fallen_backwards * ones(size(FAILURE_MODE(:,:,b)))) );

        figure();
        title (sprintf('Number of Bounces Completed for phi=%.2f', PHI_0S(1,1,b) ));
        xlabel ('l-dot_0');
        ylabel ('\phi-dot_0');

        hold on;
        pcolor (L_DOT_0S(:,:,b), PHI_DOT_0S(:,:,b), NUM_BOUNCES_COMPLETED(:,:,b));
        plot (L_DOT_0S(1, J_fwd, b), PHI_DOT_0S(I_fwd, 1, b), 'wx', 'MarkerSize', 5, 'MarkerFaceColor', 'white');
        plot (L_DOT_0S(1, J_bwd, b), PHI_DOT_0S(I_bwd, 1, b), 'wo', 'MarkerSize', 5, 'MarkerFaceColor', 'white');
        shading interp;
        colorbar;
        hold off;
    end
end