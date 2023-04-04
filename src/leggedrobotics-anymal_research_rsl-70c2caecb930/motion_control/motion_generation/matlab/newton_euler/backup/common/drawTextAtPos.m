function handle = drawTextAtPos(pos, str)
    handle = text(pos(1)+rand/5,pos(2)+rand/5,pos(3)+rand/5, ...
                  str, 'interpreter', 'latex');
end