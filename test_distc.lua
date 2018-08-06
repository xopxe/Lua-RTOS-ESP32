local apds = assert(require('apds9960'))

local min_sat = 50
local min_val = 20
local max_val = 200

local colors = {
  {"red", 0, 60},
  {"yellow", 60, 120},
  {"green", 120, 180},
  {"cyan", 180, 240},
  {"blue", 240, 300},
  {"magenta", 300, 360},
}

assert(apds.init())
local distC = apds.proximity
assert(distC.enable())

ms = ms or 100

thershold = threshold or 250

histeresis = histeresis or 3




-- callback for distC.get_dist_thresh
-- will be called when the robot is over threshold high
dump_dist = function(b)
  print('State:', b)
end

-- enable distC change monitoring
distC.get_dist_thresh(ms, thershold, histeresis, dump_dist)
