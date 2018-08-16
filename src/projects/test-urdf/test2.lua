
local ffi = require("ffi")
uv = require("uv")
gbullet  = require( "ffi/bullet3capi" )
local timer = require('timer') 

------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------

local simApp = {}


function simApp:createWheel( x, y, z, r, w )

    local radius    = r
    local width     = w

    local col = ffi.new( "double[4]", { [0]=1.0, 0.0, 0.0, 1.0} )
    local pos = ffi.new( "double[3]", { [0]=x, y, z } )
    local frameP = ffi.new( "double[3]", { [0]=0.0, 0.0, 0.0 } )
    local rotflip = ffi.new( "double[4]", { [0]=0.7071, 0.0, 0.0, 0.7071} )
    local rot = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0} )
    local frameR = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0 } )

    local cmd = gbullet.b3CreateCollisionShapeCommandInit( self.client )
    local cindex = gbullet.b3CreateCollisionShapeAddSphere( cmd, radius )
    --gbullet.b3CreateCollisionShapeSetChildTransform( cmd, cindex, pos, rot )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)

    local cmd = gbullet.b3CreateVisualShapeCommandInit( self.client )
    --local index = gbullet.b3CreateVisualShapeAddCylinder( cmd, radius, width )
    local index = gbullet.b3CreateVisualShapeAddSphere( cmd, radius )
    --gbullet.b3CreateVisualShapeSetChildTransform( cmd, index, pos, rot )
    gbullet.b3CreateVisualShapeSetRGBAColor( cmd, index, col )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    
    p("Visual Object Created:", gbullet.b3GetStatusType(status) == gbullet.CMD_CREATE_VISUAL_SHAPE_COMPLETED)
    
    local cmd = gbullet.b3CreateMultiBodyCommandInit(self.client)
    local rigidbody = gbullet.b3CreateMultiBodyBase( cmd, 2.0, cindex, index, pos, rot, frameP, frameR )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    
    p("Multibody Created:", gbullet.b3GetStatusType(status) )
        
    return rigidbody
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

    vehicle = {}

    -- Make a vehicle from components
    -- Wheel colliders first 
    vehicle.wheels = {}
    
    -- gbullet.b3CreateVisualShapeAddCylinder( cmd, radius, width )
    -- vehicle.wheels.FR = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
    -- gbullet.b3CreateVisualShapeAddCylinder( cmd, radius, width )
    -- vehicle.wheels.RL = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
    -- gbullet.b3CreateVisualShapeAddCylinder( cmd, radius, width )
    -- vehicle.wheels.RR = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
 

    --gbullet.b3LoadUrdfCommandSetStartPosition(self.car.physics, 0.0, 10.0, 0.0)
    vehicle.wheels.FL = self:createWheel(1.0, 0.0, 1.0, 0.5, 0.25)    
    p("Adding Vehicle...", vehicle.wheels.FL)
   
    -- Set sim to unitialised.
    self.simInit = 0

    ------------------------------------------------------------------------------------------------------------
    -- Install timer update - this will call simUpdate often.
    self.sdltimer = self.timer.setInterval(5, self.Update, self)
end 

------------------------------------------------------------------------------------------------------------

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

    -- Something bad has happend - weve lost client connect etc
    --local cmd = gbullet.b3ProcessServerStatus(self.client)
    --local stats = gbullet.b3GetStatusType(cmd) 
    --p( "stats", stats )
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
