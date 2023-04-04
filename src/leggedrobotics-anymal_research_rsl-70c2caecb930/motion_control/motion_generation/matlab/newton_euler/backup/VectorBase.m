classdef VectorBase
   properties
      % Name
      name_;
       
      % The handle to its drawable object.
      fig_handle_;
      
      % The vector's position.
      pos_;
      
      % The vector's value.
      vec_;
   end
   methods
      function obj = VectorBase(name, fig_handle, pos, vec)
          obj.name_ = name;
          obj.fig_handle_ = fig_handle;
          obj.pos_ = pos;
          obj.vec_ = vec;
      end
      function r = roundOff(obj)
         r = round([obj.Value],2);
      end
      function r = multiplyBy(obj,n)
         r = [obj.Value] * n;
      end
      function drawArrowAtPosition(h,pos,vec)
        handle = quiver3(h,pos(1),pos(2),pos(3),...
                           vec(1),vec(2),vec(3),0,...
                           'linewidth',2);
      end
      
      function setData(obj, pos, vec)
        set(obj.fig_handle_,...
            'xdata',pos(1),'ydata',pos(2),'zdata',pos(3),...
            'udata',vec(1),'vdata',vec(2),'wdata',vec(3));
        set(dataHandle.label,'Position',pos+vec);
      end
      
      function handle = drawTextAtPos(pos, str)
        handle = text(pos(1)+rand/5,pos(2)+rand/5,pos(3)+rand/5, ...
                          str, 'interpreter', 'latex');
      end
      function updateDataInHandle(dataHandle, pos, vec)
        set(dataHandle.h,...
              'xdata',pos(1),'ydata',pos(2),'zdata',pos(3),...
              'udata',vec(1),'vdata',vec(2),'wdata',vec(3));
          set(dataHandle.label,'Position',pos+vec);
        end
   end
end