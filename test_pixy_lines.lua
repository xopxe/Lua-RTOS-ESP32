pixy = require 'pixy2'
pixy.init()

p = function(vector, intersects, bars)
if vector then 
print('vector ('..vector.x0..','..vector.y0..')->('..vector.x1..','..vector.y1..') '..vector.index)
end

for i, ii in ipairs(intersects or {}) do
print('intersect '..i..' ('..ii.x..','..ii.y..') '..' ['..ii.n..']')
for _, v in pairs(ii.int_lines) do
print('   ', v.index, v.angle)
end
end

for i, bb in ipairs(bars or {}) do
print('barcode '..i..' '..bb.code..' ('..bb.x..','..bb.y..')' )
end
end


pixy.line.set_lines_callback(p)
pixy.line.enable(100, true, true, true)


