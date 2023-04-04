% Setup visualization
popupmenu_contents = {'I_r_IG', 'I_w_IG', 'f_ext', 'f_lf', 'f_rf', 'f_lh', 'f_rh'};
figure_handler = setupVisualization(popupmenu_contents);
% figure_handler.mass_slider.Value = data.m;
% 
% name = popupmenu_contents{get(figure_handler.dd_menu, 'Value')};
% vec = eval(['data.' name ';']);
% figure_handler.vec_x_slider.Value = vec(1);
% figure_handler.vec_y_slider.Value = vec(2);
% figure_handler.vec_z_slider.Value = vec(3);
% figure_handler.vec_x_slider_text.String = num2str(vec(1));
% figure_handler.vec_y_slider_text.String = num2str(vec(2));
% figure_handler.vec_z_slider_text.String = num2str(vec(3));
% 
% data.figure_handler = figure_handler;