function handle = drawArrowAtPosition(pos,vec)
   handle = quiver3(pos(1),pos(2),pos(3),...
                    vec(1),vec(2),vec(3),0,...
                    'linewidth',2);
end