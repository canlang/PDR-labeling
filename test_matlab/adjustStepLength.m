function [locHis, newLoc] = adjustStepLength(startTime, endTime, locHis, ratio)
%     startTime = turnTime - 10000;
%     endTime = turnTime;
    [loc, idx1] = getStepLoc(locHis,startTime);    
    [~, idx2] = getStepLoc(locHis,endTime);
    
    x = loc(1,2);
    y = loc(1,3);    
    newLoc = bsxfun(@plus, locHis(idx1:idx2,2:3)*ratio, [x-x*ratio,y-y*ratio]);
    newLoc(ismember(newLoc,[x-x*ratio,y-y*ratio],'rows'),:) = 0;
    locHis(idx1:idx2,2:3) = newLoc;
    newLoc = newLoc(all(newLoc,2),:);
end