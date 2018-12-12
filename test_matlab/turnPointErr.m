function err = turnPointErr(scale, locHis, turnTime, interval, interLoc)
    turnLoc = getStepLoc(locHis,turnTime);              %% START : ADJUSTMENT WINDOW 
    startLoc = getStepLoc(locHis,turnTime-interval);    %% END   : ADJUSTMENT WINDOW 
    hold on 
    plot(startLoc(2)/.1219,370-startLoc(3)/.1219,'xg',...
        'linewidth',1.3,'MarkerSize',12);
    plot(turnLoc(2)/.1219,370-turnLoc(3)/.1219,'xc',...
        'linewidth',1.3,'MarkerSize',12);
    hold off
    x = startLoc(2);
    y = startLoc(3);
    newTurnLoc = turnLoc(2:3)*scale + [x-x*scale,y-y*scale];
    err = norm(interLoc - newTurnLoc);
end