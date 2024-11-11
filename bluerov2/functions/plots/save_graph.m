function save_graph(save_graph_flag, base_path_for_fig_save)
	if save_graph_flag
		figure_title = get(gcf, 'Name');
		saveas(gcf, fullfile(base_path_for_fig_save, figure_title + ".eps"), 'epsc');
	end
end