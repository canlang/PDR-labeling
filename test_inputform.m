clear, close all;
pathFolder = 'input_rawdata';
% pathFolder = 'input_rawdata';
d = dir(pathFolder);
isub = [d(:).isdir]; %# returns logical vector
nameFolds = {d(isub).name}';
regexp(nameFolds, '\d{1,}\D*')
% nameFolds(~cellfun(@isempty,regexp(nameFolds, '\d{2,}\D*'))) = [];

% nameFolds(ismember(nameFolds,{'.','..',})) = [];