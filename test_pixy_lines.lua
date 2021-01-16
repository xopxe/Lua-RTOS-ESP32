pixy = require 'pixy2'
pixy.init()

p = function(lines, intersects, bars)
for i, ll in ipairs(lines or {}) do
print('line '..i..' ('..ll.x0..','..ll.y0..')->('..ll.x1..','..ll.y1..') '..ll.index..' '..tostring(ll.has_intersection))
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


