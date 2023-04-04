function popupmenu_cb(hObject, ~)
  figure_handler = evalin('caller','figure_handler');
  popupmenu_contents = evalin('caller','popupmenu_contents');
  name = popupmenu_contents{get(figure_handler.dd_menu, 'Value')};
  vec = evalin('caller', ['data.' name ';']);
  figure_handler.vec_x_slider.Value = vec(1);
  figure_handler.vec_y_slider.Value = vec(2);
  figure_handler.vec_z_slider.Value = vec(3);
  evalin('caller',['figure_handler.vec_x_slider_text.String = ' num2str(vec(1)); ]);
  evalin('caller',['figure_handler.vec_y_slider_text.String = ' num2str(vec(2)); ]);
  evalin('caller',['figure_handler.vec_z_slider_text.String = ' num2str(vec(3)); ]);
end