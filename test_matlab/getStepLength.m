function len = getStepLength(locHis)
    locHis = unique(locHis(all(locHis,2),:),'rows');
    dMat = squareform(pdist(locHis));
    len = mean(diag(dMat(2:end,1:end-1)));
end
