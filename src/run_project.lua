
if #args ~= 3 then 
    p( "Error:[Incorrect number of arguments]")
    p( "     Usage: run_project <path to project> <lua file to run>" )
    p( "     Example: run_project test-urdf test1.lua")
    os.exit()
end

package.path = package.path..";ffi/?.lua;ffi/?/init.lua"
package.path = package.path..";deps/?.lua;deps/?/init.lua;deps/stream/?.lua"
package.path = package.path..";deps/tls/?.lua;deps/path/?.lua"

local pathJoin = require('luvi').path.join

local spath = require('path')
local apppath = spath.resolve('.')

p(package.cpath)
p(apppath)

folder = pathJoin("projects", args[2])
package.path = package.path..";"..folder.."/?.lua"
filename = pathJoin(folder, args[3])

dofile( pathJoin(apppath, filename) )
