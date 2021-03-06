
local ffi = require("ffi")
uv = require("uv")
gbullet  = require( "ffi/bullet3capi" )
local timer = require('timer') 

------------------------------------------------------------------------------------------------------------

local simApp = {}

------------------------------------------------------------------------------------------------------------

local Cred = ffi.new( "double[4]",{ [0]=1.0, 0.0, 0.0, 1.0 } )
local Cgreen = ffi.new( "double[4]",{ [0]=0.0, 1.0, 0.0, 1.0 } )
local Cblue = ffi.new( "double[4]",{ [0]=0.0, 0.0, 1.0, 1.0 } )
local Cyellow = ffi.new( "double[4]",{ [0]=1.0, 1.0, 0.0, 1.0 } ) 
local Ccyan = ffi.new( "double[4]",{ [0]=0.0, 1.0, 1.0, 1.0 } )

------------------------------------------------------------------------------------------------------------


function simApp:makeBall( x, y, z, r, col )

    local radius    = r

    local pos = ffi.new( "double[3]", { [0]=x, y, z } )
    local frameP = ffi.new( "double[3]", { [0]=0.0, 0.0, 0.0 } )
    local rotflip = ffi.new( "double[4]", { [0]=0.7071, 0.0, 0.0, 0.7071} )
    local rot = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0} )
    local frameR = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0 } )

    local cmd = gbullet.b3CreateVisualShapeCommandInit( self.client )
    local shapeidx = gbullet.b3CreateVisualShapeAddSphere( cmd, radius )
    gbullet.b3CreateVisualShapeSetRGBAColor( cmd, shapeidx, col )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    p("Visual Shape index:", shapeidx)

    local cmd = gbullet.b3CreateCollisionShapeCommandInit( self.client )
    gbullet.b3CreateCollisionShapeAddSphere( cmd, radius )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local pid = gbullet.b3GetStatusCollisionShapeUniqueId(status)
    p("Collider index:", pid)

    local cmd = gbullet.b3CreateMultiBodyCommandInit(self.client)
    gbullet.b3CreateMultiBodyBase( cmd, 1.0, pid, shapeidx, pos, rot, frameP, frameR )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local rid = gbullet.b3GetStatusBodyIndex(status)

    p("Multibody Created:", gbullet.b3GetStatusType(status) )
        
    return rid
end

------------------------------------------------------------------------------------------------------------
-- simStartup - general initialisation

function simApp:Startup()

    self.timer = require('timer') 

    ------------------------------------------------------------------------------------------------------------
    -- Start the GUI Server -- need to kill it on exit too.
    if serverLaunched == nil then
        --os.execute('start cmd /k call "bin\\bullet3SharedMemory.exe"')
        os.execute('start cmd /k call "bin\\bullet3SharedMemory_GUI.exe"')
        self.timer.sleep(1000)
        serverLaunched = true
    end

    ------------------------------------------------------------------------------------------------------------

    self.client = gbullet.b3ConnectSharedMemory( 12347 )
    self.timer.sleep(500)

    -- Setup the viz
    local cmd = gbullet.b3InitConfigureOpenGLVisualizer(self.client)
    gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_GUI, 1)
    gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_TINY_RENDERER, 1)
    gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_RENDERING, 1)

    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);


    --local cmd = gbullet.b3InitResetSimulationCommand(self.client)
    --local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
    ------------------------------------------------------------------------------------------------------------
    -- Some timers for use later
    self.time_start = os.time()
    self.time_last = os.clock()

    ------------------------------------------------------------------------------------------------------------
    -- Now load in some models.
    world = {}

    -- Load the world urdf
    local cmd = gbullet.b3LoadUrdfCommandInit( self.client, "..\\..\\data\\plane\\plane.urdf")
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
    --gbullet.b3LoadUrdfCommandSetStartPosition(self.world.physics, 0.0, 0.0, 0.0)
    world.physId = gbullet.b3GetStatusBodyIndex(status)
    p("Adding world...", world.physId)   

    ------------------------------------------------------------------------------------------------------------
    -- Note: The visual shape doesnt seem to seperately instance for each object
    --       so each object shares the same visual shape because they are identical
    balls = {}
    balls.test1 = self:makeBall(1.0, 0.0, 1.5, 0.5, Cred)    
    p("Adding Ball...", balls.test1)
    balls.test2 = self:makeBall(0.0, 0.0, 1.0, 0.5, Cgreen)    
    p("Adding Ball...", balls.test2)
    balls.test3 = self:makeBall(-1.0, 0.0, 0.5, 0.5, Cblue)    
    p("Adding Ball...", balls.test3)
    -- Set sim to unitialised.
    self.simInit = 0

    ------------------------------------------------------------------------------------------------------------
    -- Install timer update - this will call simUpdate often.
    self.sdltimer = self.timer.setInterval(5, self.Update, self)
end 

------------------------------------------------------------------------------------------------------------
-- This SIM INIT method isnt ideal, but it will do for now.

function simApp:Update( ) 

    -- On the first step do some initialisation - seems to be necessary here.
    if self.simInit == 0 and self.client ~= nil then
        if gbullet.b3CanSubmitCommand(self.client) == 1 then
            self.simInit = 1
            local cmd = gbullet.b3InitPhysicsParamCommand(self.client)
            gbullet.b3PhysicsParamSetGravity(cmd, 0.0, 0.0, -9.8)
            gbullet.b3PhysicsParamSetTimeStep(cmd, 0.016)
            gbullet.b3PhysicsParamSetRealTimeSimulation(cmd, 1)
            gbullet.b3PhysicsParamSetNumSubSteps(cmd, 6)
            local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)        
        end
    end

    -- Physics updates - this will go in a coroutine.. to make it nice to run
    local time_current = os.clock()
    local dtime = time_current - self.time_last
    self.time_last = time_current

    if gbullet.b3CanSubmitCommand(self.client) == 1 then
        -- Update the physics
        local cmd = gbullet.b3InitStepSimulationCommand(self.client)
        local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    end
end

------------------------------------------------------------------------------------------------------------

coroutine.wrap( function() 
    simApp:Startup()
end)()

uv.run()

------------------------------------------------------------------------------------------------------------

p("Closing...")

------------------------------------------------------------------------------------------------------------

gbullet.b3DisconnectSharedMemory(client)

------------------------------------------------------------------------------------------------------------
