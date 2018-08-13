# luvit-bullet3
This is a simple bullet3 testing toolkit using a number of simple lua based systems. They are:
- luvit    https://github.com/luvit/luvit   A nodejs styled web toolkit using luajit
- luajit   http://luajit.org/    A jit based lua system. Fast. Fast. And it has ffi. 
- bullet3  https://github.com/bulletphysics/bullet3   A physics engine, and probably the best open sourced available one. 

With these combined systems, the user has access to web capabilities, the bullet3 C API and it can be extended easily with luajit's ffi. 

Luvit and Luajit packages are MIT licensed. 
Bullet3 is zlib licensed.

Please check before using though if you have any license questions please examine the package sources explanations on licensing.

## Just Run It
All you need to run the whole thing is:

1. A PC with Win10 and an x64 CPU. 

2. A command prompt running from the /src folder:

`.\bin\luvit.exe .\run_project.lua test-urdf test1.lua`

Enjoy. :)