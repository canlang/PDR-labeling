function err = turnPointErr2(x,y, locHis, turnTime, interval, interLoc)
%%% for OPTIMIZE HEADING AND STEP LENGTH
%     scale   = x(1);
%     heading = x(2);
    scale   = x;
    heading = y;
    % FOR ADJUSTMENT WINDOW
    turnLoc = getStepLoc(locHis,turnTime);              %% END  
    strtLoc = getStepLoc(locHis,turnTime-interval);     %% START
    
%     hold on 
%     plot(startLoc(2)/.1219,370-startLoc(3)/.1219,'xg',...
%         'linewidth',1.3,'MarkerSize',12);
%     plot(turnLoc(2)/.1219,370-turnLoc(3)/.1219,'xc',...
%         'linewidth',1.3,'MarkerSize',12);
%     hold off

    x0 = strtLoc(2);
    y0 = strtLoc(3);
    newTurnLoc = turnLoc(2:3)*scale + [x0-x0*scale,y0-y0*scale];
%     newStrtLoc = strtLoc(2:3)*scale + [x-x*scale,y-y*scale];
    err1 = norm(interLoc - newTurnLoc);
    
    turnLoc = turnLoc(2:3);
    strtLoc = strtLoc(2:3);
    turnLoc = turnLoc - strtLoc;
    turnLoc = [...
        cosd(heading)*turnLoc(1)-sind(heading)*turnLoc(2),...
        sind(heading)*turnLoc(1)+cosd(heading)*turnLoc(2)];
    newTurnLoc = turnLoc + strtLoc;
    err2 = norm(interLoc - newTurnLoc);
    
    err = err1 + err2;
end