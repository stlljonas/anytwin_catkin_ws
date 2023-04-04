classdef DrawableVector
 properties
   % Name
   name_;

   % The handle to its parent axes.
   axes_handle_;

   % The vector's position.
   pos_;

   % The vector's value.
   vec_;

   % The handle to the drawn object.
   vec_handle_;

   % The handle to the text label of the drawn object.
   label_handle_;
  end
  methods
    function obj = DrawableVector(axes_handle, name, pos, vec)
      obj.name_ = name;
      obj.axes_handle_ = axes_handle;
      obj.pos_ = pos;
      obj.vec_ = vec;

      obj.draw();
    end
    function obj = draw(obj)
      obj.vec_handle_ = ...
          quiver3(obj.axes_handle_, ...
                  obj.pos_(1),obj.pos_(2),obj.pos_(3),...
                  obj.vec_(1),obj.vec_(2),obj.vec_(3),0,...
                  'linewidth',2);
                
      labelOffset = norm(obj.vec_)*0.1;
      obj.label_handle_ = ...
          text(obj.pos_(1)+obj.vec_(1)+labelOffset, ...
               obj.pos_(2)+obj.vec_(2)+labelOffset, ...
               obj.pos_(3)+obj.vec_(3)+labelOffset, ...
               obj.name_, 'interpreter', 'latex');
    end

    function obj = remove(obj)
      delete(obj.vec_handle_);
      delete(obj.label_handle_);
    end

    function obj = setPositionAndValues(obj, pos, vec)
      obj.pos_ = pos;
      obj.vec_ = vec;
      obj.draw();
    end
 end
end