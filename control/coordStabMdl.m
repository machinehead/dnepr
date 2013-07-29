function [ ts, pos, ps, spds, trg_spds, loc_errs, raps ] = coordStabMdl()

ts = [];
pos = [];
ps = [];
spds = [];
trg_spds = [];
loc_errs = [];
raps = [];

homePos = [100 150];
lastPos = homePos;
currPos = lastPos + [50 50];
act_spd = [0 0];
loc_error = [0 0];
p = [0 0];

real_act_spd = [0 0];
real_act_pos = currPos;

t = 0;
while t < 10000
    % obtain
    real_act_spd = real_act_spd + p .* [-1 1];
    real_act_pos = real_act_pos + real_act_spd * 0.01;
    
    if mod(t,30) == 0
        currPos = real_act_pos;

        % calc all
        act_spd = ([-1 1] .* (currPos - lastPos) + act_spd) .* 0.5;
        lastPos = currPos;

        loc_error = [-1 1] .* (homePos - currPos);
        tar_spd = 0.3 .* loc_error;
        rate_err = tar_spd - act_spd;
        p = max(min(4 * rate_err, 1), -1);
    
    end;
    
    % log
    ts = [ts; t];
    pos = [pos; currPos];
    ps = [ps; p];
    spds = [spds; act_spd];
    trg_spds = [trg_spds; tar_spd];
    loc_errs = [loc_errs; loc_error];
    raps = [raps; real_act_pos];
    
    t = t + 1;
end;

end

