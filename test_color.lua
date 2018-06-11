v=require('apds9660')
assert(v.init())

ms = ms or 1000

-- callback for get_color_continuous
-- will be called with (r,g,b,a [,h,s,v])
-- hsv will be provided if enabled on get_color_continuous
-- r,g,b,a : 16 bits
-- h: 0..360
-- s,v: 0..255
dump_rgb = function(a,r,g,b,h,s,v) 
  print('ambient', a, 'rgb', r, g, b,'hsv', h, s, v) 
end

-- callback for get_color_change
-- will be called with (color, s, v)
-- color: one of "red", "yellow", "green", "cyan", "blue", "magenta"
-- s,v: 0..255
dump_color_change = function(color, s, v) 
  print('color', color, 'sv', s, v) 
end

-- enable raw color monitoring, enable hsv mode
v.get_color_continuous(ms, dump_rgb, true)

-- enable color change monitoring, enable hsv mode
v.get_color_change(ms, dump_color_change)

-- run for 60 seconds
tmr.sleepms(60*1000)

-- stop monitoring distances
v.get_color_continuous(false)
v.get_color_change(false)