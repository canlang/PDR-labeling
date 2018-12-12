function err = turnPointErr4(scale, hDelta, locHis, turnTime, interval, interLoc,R)
%%% for OPTIMIZE HEADING AND STEP LENGTH
%%% version3 - fix: ADD ERROR from ROAD NETWORK
   
%     scale   = x(1);         % STEP LENGTH SCALE DELTA
%     hDelta = x(2);          % HEADING DELTA
    %     scale   = x;
    %     heading = y;
    % FOR ADJUSTMENT WINDOW
    [turnLoc, eIdx] = getStepLoc(locHis,turnTime);              %% END
    [strtLoc, sIdx] = getStepLoc(locHis,turnTime-interval);     %% START

    %     hold on
    %     plot(startLoc(2)/.1219,370-startLoc(3)/.1219,'xg',...
    %         'linewidth',1.3,'MarkerSize',12);
    %     plot(turnLoc(2)/.1219,370-turnLoc(3)/.1219,'xc',...
    %         'linewidth',1.3,'MarkerSize',12);
    %     hold off

    x0 = strtLoc(2);
    y0 = strtLoc(3);
    
    [rIdx, ~] = my1nnIntersection(R,strtLoc(2:3));
    projP0 = project_point_to_line_segment(R(rIdx,[1,2]),R(rIdx,[3,4]), strtLoc(2:3));
    xp0 = projP0(1);
    yp0 = projP0(2);
    delta = [xp0-x0, yp0-y0];
    if sIdx > eIdx
        disp('***error***: turnPointErr4, unexpected case!!');
    end
    projLoc = bsxfun(@plus,locHis(sIdx:eIdx, 2:3),delta);
    temp = projLoc(1,:);
%     for i = 1:size(projLoc,1)
%         projLoc(i,:) = adjustRotation(projLoc(1,1),projLoc(1,2),projLoc(i,1),projLoc(i,2),heading);
%     end
    [projLoc(:,1),projLoc(:,2)] = arrayfun(...
        @(x,y) adjustRotation(projLoc(1,1),projLoc(1,2),x,y,hDelta),...
        projLoc(:,1),projLoc(:,2));
    
    if projLoc(1,:) ~= temp
        disp('***error***: turnPointErr4, unexpected case!!');
    end
    err3 = zeros(1,size(projLoc,1));
    for i = 1:size(projLoc,1)
        [~, dist] = my1nnIntersection(R,projLoc(i,:));
        err3(i) = dist;
    end
    err3 = sum(err3);
    
    
    newTurnLoc = turnLoc(2:3)*scale + [x0-x0*scale,y0-y0*scale];
    %     newStrtLoc = strtLoc(2:3)*scale + [x-x*scale,y-y*scale];
    err1 = norm(interLoc - newTurnLoc);

    turnLoc = turnLoc(2:3);
    strtLoc = strtLoc(2:3);
    turnLoc = turnLoc - strtLoc;
    turnLoc = [...
        cosd(hDelta)*turnLoc(1)-sind(hDelta)*turnLoc(2),...
        sind(hDelta)*turnLoc(1)+cosd(hDelta)*turnLoc(2)];
    newTurnLoc = turnLoc + strtLoc;
    err2 = norm(interLoc - newTurnLoc);

    err = err1 + err2 + err3;
end