-- { {xshutpin, [newadddr]}, ... }
local sensors = {
 {16},
 {17},
 {2},
 {14},
 {12},
 {13},
}


v=require('vl53ring')
assert(v.init(sensors))

-- faster, less precise measuremente
v.set_measurement_timing_budget(5000);

ms = ms or 1000  -- period of distance measurements

-- the callback will be called with all sensor readings
local dist_callback= function(d1, d2, d3, d4, d5, d6)
    print('dist:', d1, d2, d3, d4, d5, d6)
end

-- start monitoring distances
v.get_continuous(ms, dist_callback)

-- run for 60 seconds
tmr.sleepms(60*1000)

-- stop monitoring distances
v.get_continuous(false)
