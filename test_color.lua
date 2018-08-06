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
local color = apds.color
assert(color.set_color_table(colors))
assert(color.set_sv_limits(min_sat,min_val,max_val))
assert(color.enable())

ms = ms or 100



-- callback for color.get_continuous
-- will be called with (r,g,b,a [,h,s,v])
-- hsv will be provided if enabled on color.get_continuous
-- r,g,b,a : 16 bits
-- h: 0..360
-- s,v: 0..255
dump_rgb = function(r,g,b,a,h,s,v, name)
  print('ambient:', a, 'rgb:', r, g, b,'hsv:', h, s, v, 'name:', name)
end

-- callback for get_change
-- will be called with (color, s, v)
-- color: one of "red", "yellow", "green", "cyan", "blue", "magenta"
-- s,v: 0..255
dump_color_change = function(c, s, v)
  print('color', c, 'sv', s, v)
end


--power on led
local led_pin = pio.GPIO32
pio.pin.setdir(pio.OUTPUT, led_pin)
pio.pin.sethigh(led_pin)

-- enable raw color monitoring, enable hsv mode
--color.get_continuous(ms, dump_rgb, true)

-- enable color change monitoring, enable hsv mode
color.get_change(ms, dump_color_change)

--[[
while true do
  tmr.sleepms(50)
  print (color.get_rgb())
end
--]]

-- run for 60 seconds
tmr.sleepms(60*1000)

-- stop monitoring distances
color.get_continuous(false)
color.get_change(false)

pio.pin.setlow(led_pin)
