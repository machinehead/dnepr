function [ sums ] = accSum( accs, decay )
    sums = [];
    sum = zeros(1,size(accs,2));
    
    for i = 1:size(accs,1)
        sum = sum * decay + accs(i,:);
        sums = [sums; sum];
    end
end

