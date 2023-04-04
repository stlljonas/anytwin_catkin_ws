function slider_cb(hObject, ~, dim)
  figure_handler = evalin('caller','figure_handler');
  popupmenu_contents = evalin('caller','popupmenu_contents');
  name = popupmenu_contents{get(figure_handler.dd_menu, 'Value')};
  evalin('caller', ['data.' name '(' num2str(dim) ') = ' num2str(hObject.Value) ';']);
  evalin('caller', 'data = update_ne(data);');
  evalin('caller', 'updateHandles(data);');
  evalin('caller',['figure_handler.vec_x_slider_text.String = '...
                   'num2str(figure_handler.vec_x_slider.Value); ']);
  evalin('caller',['figure_handler.vec_y_slider_text.String = '...
                   'num2str(figure_handler.vec_y_slider.Value); ']);
  evalin('caller',['figure_handler.vec_z_slider_text.String = '...
                   'num2str(figure_handler.vec_z_slider.Value); ']);
end