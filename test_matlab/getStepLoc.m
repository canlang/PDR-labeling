function [loc,idx] = getStepLoc(locHis, time)
%     locHis = locHis(all(locHis,2),:);
    [IDX, ~] = knnsearch(locHis(:,1), time, 'k',1);
    loc = locHis(IDX, :);
    idx = IDX;
end