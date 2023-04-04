function slider_mass_cb(hObject, ~)
  evalin('caller', ['data.m = ' num2str(hObject.Value) ';']);
  evalin('caller', 'data = update_ne(data);');
  evalin('caller', 'updateHandles(data);');
  evalin('caller',['figure_handler.mass_slider_text.String = ' num2str(hObject.Value); ]);
end