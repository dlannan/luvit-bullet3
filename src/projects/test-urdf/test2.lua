
local ffi = require("ffi")
uv = require("uv")
gbullet  = require( "ffi/bullet3capi" )
local timer = require('timer') 

------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------

local simApp = {}


function simApp:createWheel( x, y, z, r, w, mass)

    local radius    = r
    local width     = w

    local col = ffi.new( "double[4]", { [0]=0.0, 1.0, 0.0, 1.0} )
    local pos = ffi.new( "double[3]", { [0]=x, y, z } )
    local frameP = ffi.new( "double[3]", { [0]=0.0, 0.0, 0.0 } )
    local rotflip = ffi.new( "double[4]", { [0]=0.7071, 0.0, 0.0, 0.7071} )
    local rot = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0} )
    local frameR = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0 } )

    local cmd = gbullet.b3CreateCollisionShapeCommandInit( self.client )
    local cindex = gbullet.b3CreateCollisionShapeAddCylinder( cmd, radius, width )
    -- if cid ~= nil then 
    --     gbullet.b3CreateCollisionShapeSetChildTransform( cmd, cid, pos, rot )
    -- end
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local cid = gbullet.b3GetStatusCollisionShapeUniqueId(status)

    local cmd = gbullet.b3CreateVisualShapeCommandInit( self.client )
    local vindex = gbullet.b3CreateVisualShapeAddCylinder( cmd, radius, width )
    -- if vid ~= nil then     
    --     gbullet.b3CreateVisualShapeSetChildTransform( cmd, vid, pos, rot )
    -- end
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local vid = gbullet.b3GetStatusVisualShapeUniqueId(status)
    gbullet.b3CreateVisualShapeSetRGBAColor( cmd, vid, col )

    p("Visual Object Created:", gbullet.b3GetStatusType(status) == gbullet.CMD_CREATE_VISUAL_SHAPE_COMPLETED)
    
    local cmd = gbullet.b3CreateMultiBodyCommandInit(self.client)
    local rigidbody = gbullet.b3CreateMultiBodyBase( cmd, mass, cid, vid, pos, rotflip, frameP, frameR )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    
    local bidx = gbullet.b3GetStatusBodyIndex( status )
    p("Multibody Created:", gbullet.b3GetStatusType(status) == gbullet.CMD_CREATE_MULTI_BODY_COMPLETED )
    
    return bidx, cid, vid
end

------------------------------------------------------------------------------------------------------------

function simApp:createFixedWheelJoint( pb, cb, px, py, pz, cx, cy, cz )

    local pos = ffi.new( "double[3]", { [0]=0.0, 0.0, 0.0 } )
    local rot = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0} )
    local fchild = ffi.new( "double[7]", { [0]=cx, cy, cz, 0.0, 0.0, 0.0, 1.0 } )
    local fparent = ffi.new( "double[7]", { [0]=px, py, pz, 0.0, 0.0, 0.0, 1.0 } )
    local axis = ffi.new( "double[3]", { [0]=0.0, 0.0, 1.0} )

    local joint = ffi.new("struct b3JointInfo[1]")
    joint[0].m_jointType = gbullet.ePoint2PointType
    joint[0].m_jointAxis = axis
    joint[0].m_jointName = "joint"..cb
    joint[0].m_linkName = "link"..cb
    joint[0].m_parentFrame = fparent
    joint[0].m_childFrame = fchild
    joint[0].m_parentIndex = pb
    joint[0].m_jointLowerLimit = 0.01 
    joint[0].m_jointUpperLimit = 0.02 

    local cmd = gbullet.b3InitCreateUserConstraintCommand( self.client, pb, -1, cb, -1, joint )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
  
    return bl 
end

------------------------------------------------------------------------------------------------------------

function simApp:createBody( x, y, z, w, h, l, mass )

    local col = ffi.new( "double[4]", { [0]=0.0, 1.0, 0.0, 1.0} )
    local extents = ffi.new( "double[3]", {[0]=w*0.5, h*0.5, l*0.5} )
    local pos = ffi.new( "double[3]", { [0]=x, y, z } )
    local frameP = ffi.new( "double[3]", { [0]=0.0, 0.0, 0.0 } )
    local rot = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0} )
    local frameR = ffi.new( "double[4]", { [0]=0.0, 0.0, 0.0, 1.0 } )

    local cmd = gbullet.b3CreateCollisionShapeCommandInit( self.client )
    local cindex = gbullet.b3CreateCollisionShapeAddBox( cmd, extents )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local cid = gbullet.b3GetStatusCollisionShapeUniqueId(status)

    local cmd = gbullet.b3CreateVisualShapeCommandInit( self.client )
    local vindex = gbullet.b3CreateVisualShapeAddBox( cmd, extents )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    local vid = gbullet.b3GetStatusVisualShapeUniqueId(status)
    gbullet.b3CreateVisualShapeSetRGBAColor( cmd, vid, col )

    p("Visual Object Created:", gbullet.b3GetStatusType(status) == gbullet.CMD_CREATE_VISUAL_SHAPE_COMPLETED)
    
    local cmd = gbullet.b3CreateMultiBodyCommandInit(self.client)
    local rigidbody = gbullet.b3CreateMultiBodyBase( cmd, mass, cid, vid, pos, rot, frameP, frameR )
    local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)
    
    local bidx = gbullet.b3GetStatusBodyIndex( status )
    p("Multibody Created:", gbullet.b3GetStatusType(status) == gbullet.CMD_CREATE_MULTI_BODY_COMPLETED )
    
    return bidx, cid, vid
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
    -- local cmd = gbullet.b3InitConfigureOpenGLVisualizer(self.client)
    -- gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_GUI, 1)
    -- gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_TINY_RENDERER, 1)
    -- gbullet.b3ConfigureOpenGLVisualizerSetVisualizationFlags( cmd, gbullet.COV_ENABLE_RENDERING, 1)

    -- local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)

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

    --gbullet.b3LoadUrdfCommandSetStartPosition(self.car.physics, 0.0, 10.0, 0.0)
    vehicle.wheels.FL = self:createWheel(2.0, 1.0, 1.0, 0.5, 0.25, 2.0)
    vehicle.wheels.FR = self:createWheel(2.0, -1.0, 1.0, 0.5, 0.25, 2.0)   

    -- rear axle split
    local laxb, laxc, laxv = self:createWheel(-2.0, 0.2, 1.0, 0.1, 0.1, 2.0)   
    local raxb, raxc, raxv = self:createWheel(-2.0, -0.2, 1.0, 0.1, 0.1, 2.0)  

    local rlb, rlc, rlv = self:createWheel(-2.0, 1.0, 1.0, 0.5, 0.25, 2.0)    
    local rrb, rrc, rrv = self:createWheel(-2.0, -1.0, 1.0, 0.5, 0.25, 2.0)   

    self:createFixedWheelJoint( laxb, rlb, 0.0, 0.0, -0.05, 0.0, 0.0, 0.125 )
    self:createFixedWheelJoint( raxb, rrb, 0.0, 0.0, 0.05, 0.0, 0.0, -0.125 )

    local bb, bc, bv = self:createBody( 0.0, 0.0, 1.0, 4.5, 1.2, 0.7, 20 )

    self:createFixedWheelJoint( bb, laxb, -2.0, 0.6, 0.0, 0.0, 0.0, 0.05 )
    self:createFixedWheelJoint( bb, raxb, -2.0, -0.6, 0.0, 0.0, 0.0, -0.05 )

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

    --local cmd = gbullet.b3InitRequestDebugLinesCommand(self.client, 1) --0x182c)
    --local status = gbullet.b3SubmitClientCommandAndWaitStatus(self.client, cmd)        
    --p("Debug Visuals: ", gbullet.b3GetStatusType(status) == gbullet.CMD_DEBUG_LINES_COMPLETED)            

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
