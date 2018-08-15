--
-- Created by David Lannan - copyright 2013
-- Developed for the Byt3D project. byt3d.codeplex.com
-- User: dlannan
-- Date: 21/04/13
-- Time: 8:28 PM
--
local ffi = require( "ffi" )

local libs = ffi_bullet_libs or {
    Windows = { x86 = "bin/bullet3CAPI.dll", x64 = "bin/bullet3CAPI.dll" },
    OSX     = { x86 = "/usr/lib/bulletCAPI.dylib", x64 = "/usr/lib/bulletCAPI.dylib" },
    Linux   = { x86 = "bulletCAPI", x64 = "bulletCAPI", arm = "bulletCAPI" },
}

local lib   = ffi_bullet_libs or libs[ ffi.os ][ ffi.arch ]
local bullet= ffi.load( lib )

ffi.cdef [[
// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
//
// Draft high-level generic physics C-API. For low-level access, use the physics SDK native API's.
// Work in progress, functionality will be added on demand.


enum EnumSharedMemoryClientCommand
{
    CMD_LOAD_SDF,
	CMD_LOAD_URDF,
	CMD_LOAD_BULLET,
	CMD_SAVE_BULLET,
	CMD_LOAD_MJCF,
    CMD_LOAD_SOFT_BODY,
	CMD_SEND_BULLET_DATA_STREAM,
	CMD_CREATE_BOX_COLLISION_SHAPE,
	CMD_CREATE_RIGID_BODY,
	CMD_DELETE_RIGID_BODY,
	CMD_CREATE_SENSOR,
	CMD_INIT_POSE,
	CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
	CMD_SEND_DESIRED_STATE,
	CMD_REQUEST_ACTUAL_STATE,
	CMD_REQUEST_DEBUG_LINES,
    CMD_REQUEST_BODY_INFO,
	CMD_REQUEST_INTERNAL_DATA,
    CMD_STEP_FORWARD_SIMULATION,
    CMD_RESET_SIMULATION,
    CMD_PICK_BODY,
    CMD_MOVE_PICKED_BODY,
    CMD_REMOVE_PICKING_CONSTRAINT_BODY,
    CMD_REQUEST_CAMERA_IMAGE_DATA,
    CMD_APPLY_EXTERNAL_FORCE,
	CMD_CALCULATE_INVERSE_DYNAMICS,
    CMD_CALCULATE_INVERSE_KINEMATICS,
    CMD_CALCULATE_JACOBIAN,
    CMD_CALCULATE_MASS_MATRIX,
    CMD_USER_CONSTRAINT,
    CMD_REQUEST_CONTACT_POINT_INFORMATION,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS,

	CMD_REQUEST_AABB_OVERLAP,

	CMD_SAVE_WORLD,
	CMD_REQUEST_VISUAL_SHAPE_INFO,
    CMD_UPDATE_VISUAL_SHAPE,
    CMD_LOAD_TEXTURE,
    CMD_SET_SHADOW,
	CMD_USER_DEBUG_DRAW,
	CMD_REQUEST_VR_EVENTS_DATA,
	CMD_SET_VR_CAMERA_STATE,
	CMD_SYNC_BODY_INFO,
	CMD_STATE_LOGGING,
    CMD_CONFIGURE_OPENGL_VISUALIZER,
	CMD_REQUEST_KEYBOARD_EVENTS_DATA,
	CMD_REQUEST_OPENGL_VISUALIZER_CAMERA,
	CMD_REMOVE_BODY,
	CMD_CHANGE_DYNAMICS_INFO,
	CMD_GET_DYNAMICS_INFO,
	CMD_PROFILE_TIMING,
	CMD_CREATE_COLLISION_SHAPE,
	CMD_CREATE_VISUAL_SHAPE,
	CMD_CREATE_MULTI_BODY,
	CMD_REQUEST_COLLISION_INFO,
	CMD_REQUEST_MOUSE_EVENTS_DATA,
	CMD_CHANGE_TEXTURE,
	CMD_SET_ADDITIONAL_SEARCH_PATH,
	CMD_CUSTOM_COMMAND,
	CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS,
	CMD_SAVE_STATE,
	CMD_RESTORE_STATE,
	CMD_REQUEST_COLLISION_SHAPE_INFO,

	CMD_SYNC_USER_DATA,
	CMD_REQUEST_USER_DATA,
	CMD_ADD_USER_DATA,
	CMD_REMOVE_USER_DATA,
    CMD_MAX_CLIENT_COMMANDS,
};

enum EnumSharedMemoryServerStatus
{
        CMD_SHARED_MEMORY_NOT_INITIALIZED=0,
        CMD_WAITING_FOR_CLIENT_COMMAND,
        CMD_CLIENT_COMMAND_COMPLETED,
        CMD_UNKNOWN_COMMAND_FLUSHED,
		CMD_SDF_LOADING_COMPLETED,
        CMD_SDF_LOADING_FAILED,
        CMD_URDF_LOADING_COMPLETED,
        CMD_URDF_LOADING_FAILED,
		CMD_BULLET_LOADING_COMPLETED,
		CMD_BULLET_LOADING_FAILED,
		CMD_BULLET_SAVING_COMPLETED,
		CMD_BULLET_SAVING_FAILED,
		CMD_MJCF_LOADING_COMPLETED,
		CMD_MJCF_LOADING_FAILED,
		CMD_REQUEST_INTERNAL_DATA_COMPLETED,
		CMD_REQUEST_INTERNAL_DATA_FAILED,
        CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,
        CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,
        CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,
        CMD_RIGID_BODY_CREATION_COMPLETED,
        CMD_SET_JOINT_FEEDBACK_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_FAILED,
        CMD_DEBUG_LINES_COMPLETED,
        CMD_DEBUG_LINES_OVERFLOW_FAILED,
        CMD_DESIRED_STATE_RECEIVED_COMPLETED,
        CMD_STEP_FORWARD_SIMULATION_COMPLETED,
        CMD_RESET_SIMULATION_COMPLETED,
        CMD_CAMERA_IMAGE_COMPLETED,
        CMD_CAMERA_IMAGE_FAILED,
        CMD_BODY_INFO_COMPLETED,
        CMD_BODY_INFO_FAILED,
		CMD_INVALID_STATUS,
		CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,
		CMD_CALCULATED_INVERSE_DYNAMICS_FAILED,
        CMD_CALCULATED_JACOBIAN_COMPLETED,
        CMD_CALCULATED_JACOBIAN_FAILED,
        CMD_CALCULATED_MASS_MATRIX_COMPLETED,
        CMD_CALCULATED_MASS_MATRIX_FAILED,
		CMD_CONTACT_POINT_INFORMATION_COMPLETED,
		CMD_CONTACT_POINT_INFORMATION_FAILED,
		CMD_REQUEST_AABB_OVERLAP_COMPLETED,
		CMD_REQUEST_AABB_OVERLAP_FAILED,
		CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED,
		CMD_CALCULATE_INVERSE_KINEMATICS_FAILED,
		CMD_SAVE_WORLD_COMPLETED,
		CMD_SAVE_WORLD_FAILED,
        CMD_VISUAL_SHAPE_INFO_COMPLETED,
        CMD_VISUAL_SHAPE_INFO_FAILED,
        CMD_VISUAL_SHAPE_UPDATE_COMPLETED,
        CMD_VISUAL_SHAPE_UPDATE_FAILED,
        CMD_LOAD_TEXTURE_COMPLETED,
        CMD_LOAD_TEXTURE_FAILED,
		CMD_USER_DEBUG_DRAW_COMPLETED,
		CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,
		CMD_USER_DEBUG_DRAW_FAILED,
		CMD_USER_CONSTRAINT_COMPLETED,
		CMD_USER_CONSTRAINT_INFO_COMPLETED,
		CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED,
        CMD_REMOVE_USER_CONSTRAINT_COMPLETED,
        CMD_CHANGE_USER_CONSTRAINT_COMPLETED,
		CMD_REMOVE_USER_CONSTRAINT_FAILED,
        CMD_CHANGE_USER_CONSTRAINT_FAILED,
		CMD_USER_CONSTRAINT_FAILED,
		CMD_REQUEST_VR_EVENTS_DATA_COMPLETED,
		CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED,
		CMD_SYNC_BODY_INFO_COMPLETED,
		CMD_SYNC_BODY_INFO_FAILED,
		CMD_STATE_LOGGING_COMPLETED,
		CMD_STATE_LOGGING_START_COMPLETED,
		CMD_STATE_LOGGING_FAILED,
		CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED,
		CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED,
		CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED,
		CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED,
		CMD_REMOVE_BODY_COMPLETED,
		CMD_REMOVE_BODY_FAILED,
		CMD_GET_DYNAMICS_INFO_COMPLETED,
		CMD_GET_DYNAMICS_INFO_FAILED,
		CMD_CREATE_COLLISION_SHAPE_FAILED,
		CMD_CREATE_COLLISION_SHAPE_COMPLETED,
		CMD_CREATE_VISUAL_SHAPE_FAILED,
		CMD_CREATE_VISUAL_SHAPE_COMPLETED,
		CMD_CREATE_MULTI_BODY_FAILED,
		CMD_CREATE_MULTI_BODY_COMPLETED,
		CMD_REQUEST_COLLISION_INFO_COMPLETED,
		CMD_REQUEST_COLLISION_INFO_FAILED,
		CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED,
		CMD_CHANGE_TEXTURE_COMMAND_FAILED,
		CMD_CUSTOM_COMMAND_COMPLETED,
		CMD_CUSTOM_COMMAND_FAILED,
		CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED,
		CMD_SAVE_STATE_FAILED,
		CMD_SAVE_STATE_COMPLETED,
		CMD_RESTORE_STATE_FAILED,
		CMD_RESTORE_STATE_COMPLETED,
		CMD_COLLISION_SHAPE_INFO_COMPLETED,
		CMD_COLLISION_SHAPE_INFO_FAILED,
		CMD_LOAD_SOFT_BODY_FAILED,
		CMD_LOAD_SOFT_BODY_COMPLETED,

		CMD_SYNC_USER_DATA_COMPLETED,
		CMD_SYNC_USER_DATA_FAILED,
		CMD_REQUEST_USER_DATA_COMPLETED,
		CMD_REQUEST_USER_DATA_FAILED,
		CMD_ADD_USER_DATA_COMPLETED,
		CMD_ADD_USER_DATA_FAILED,
		CMD_REMOVE_USER_DATA_COMPLETED,
		CMD_REMOVE_USER_DATA_FAILED,

        CMD_MAX_SERVER_COMMANDS
};

enum JointInfoFlags
{
    JOINT_HAS_MOTORIZED_POWER=1,
};

enum 
{
	COLLISION_SHAPE_TYPE_BOX=1,
	COLLISION_SHAPE_TYPE_CYLINDER_X,
	COLLISION_SHAPE_TYPE_CYLINDER_Y,
	COLLISION_SHAPE_TYPE_CYLINDER_Z,
	COLLISION_SHAPE_TYPE_CAPSULE_X,
	COLLISION_SHAPE_TYPE_CAPSULE_Y,
	COLLISION_SHAPE_TYPE_CAPSULE_Z,
	COLLISION_SHAPE_TYPE_SPHERE
};

// copied from btMultiBodyLink.h
enum JointType {
	eRevoluteType = 0,
	ePrismaticType = 1,
	eSphericalType = 2,
	ePlanarType = 3,
	eFixedType = 4,
	ePoint2PointType = 5,
	eGearType=6
};


enum b3JointInfoFlags
{
	eJointChangeMaxForce = 1,
	eJointChangeChildFramePosition = 2,
	eJointChangeChildFrameOrientation = 4,
};

typedef struct b3JointInfo
{
        char m_linkName[1024];
        char m_jointName[1024];
        int m_jointType;
        int m_qIndex;
        int m_uIndex;
        int m_jointIndex;
        int m_flags;
		double m_jointDamping;
		double m_jointFriction;
		double m_jointLowerLimit;
		double m_jointUpperLimit;
		double m_jointMaxForce;
		double m_jointMaxVelocity;
		double m_parentFrame[7]; 
		double m_childFrame[7]; 
		double m_jointAxis[3]; 
		int m_parentIndex;
};


enum UserDataValueType {
	USER_DATA_VALUE_TYPE_BYTES = 0,
	USER_DATA_VALUE_TYPE_STRING = 1,
};

typedef struct b3UserDataValue 
{
	int m_type;
	int m_length;
	char* m_data1;
};

typedef struct b3UserConstraint
{
    int m_parentBodyIndex;
    int m_parentJointIndex;
    int m_childBodyIndex;
    int m_childJointIndex;
    double m_parentFrame[7];
    double m_childFrame[7];
    double m_jointAxis[3];
    int m_jointType;
    double m_maxAppliedForce;
    int m_userConstraintUniqueId;
	double m_gearRatio;
	int m_gearAuxLink;
	double m_relativePositionTarget;
	double m_erp;
};

typedef struct b3BodyInfo
{
	char m_baseName[1024];
	char m_bodyName[1024]; 
};


enum DynamicsActivationState
{
	eActivationStateEnableSleeping = 1,
	eActivationStateDisableSleeping = 2,
	eActivationStateWakeUp = 4,
	eActivationStateSleep = 8,
};

typedef struct b3DynamicsInfo
{
	double m_mass;
	double m_localInertialDiagonal[3];
	double m_localInertialFrame[7];
	double m_lateralFrictionCoeff;

	double m_rollingFrictionCoeff;
	double m_spinningFrictionCoeff;
	double m_restitution;
	double m_contactStiffness;
	double m_contactDamping;
};

// copied from btMultiBodyLink.h
enum SensorType {
	eSensorForceTorqueType = 1,
};


typedef struct b3JointSensorState
{
  double m_jointPosition;
  double m_jointVelocity;
  double m_jointForceTorque[6];  /* note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint */
  double m_jointMotorTorque;
};

typedef struct b3DebugLines
{
    int m_numDebugLines;
    const float*  m_linesFrom;
    const float*  m_linesTo;
    const float*  m_linesColor;
};

typedef struct b3OverlappingObject
{
	int m_objectUniqueId;
	int m_linkIndex;
};

typedef struct b3AABBOverlapData
{
    int m_numOverlappingObjects;
	struct b3OverlappingObject* m_overlappingObjects;
};

typedef struct b3CameraImageData
{
	int m_pixelWidth;
	int m_pixelHeight;
	const unsigned char* m_rgbColorData;
	const float* m_depthValues;
	const int* m_segmentationMaskValues;
};

typedef struct b3OpenGLVisualizerCameraInfo
{
    int m_width;
    int m_height;
	float m_viewMatrix[16];
	float m_projectionMatrix[16];
	
	float m_camUp[3];
	float m_camForward[3];

	float m_horizontal[3];
	float m_vertical[3];
	
	float m_yaw;
	float m_pitch;
	float m_dist;
	float m_target[3];
};

typedef struct b3UserConstraintState
{
	double m_appliedConstraintForces[6];
	int m_numDofs;
};

enum b3VREventType
{
	VR_CONTROLLER_MOVE_EVENT=1,
	VR_CONTROLLER_BUTTON_EVENT=2,
	VR_HMD_MOVE_EVENT=4,
	VR_GENERIC_TRACKER_MOVE_EVENT=8,
};

static const int MAX_VR_ANALOG_AXIS = 5;
static const int MAX_VR_BUTTONS = 64;
static const int MAX_VR_CONTROLLERS = 8;


static const int MAX_KEYBOARD_EVENTS = 256;
static const int MAX_MOUSE_EVENTS = 256;

static const int MAX_SDF_BODIES = 512;


enum b3VRButtonInfo
{
	eButtonIsDown = 1,
	eButtonTriggered = 2,
	eButtonReleased = 4,
};



enum eVRDeviceTypeEnums
{
	VR_DEVICE_CONTROLLER=1,
	VR_DEVICE_HMD=2,
	VR_DEVICE_GENERIC_TRACKER=4,
};

enum EVRCameraFlags
{
	VR_CAMERA_TRACK_OBJECT_ORIENTATION=1,
};

typedef struct b3VRControllerEvent
{
	int m_controllerId;
	int m_deviceType;
	int m_numMoveEvents;
	int m_numButtonEvents;
	
	float m_pos[4];
	float m_orn[4];

	float m_analogAxis;
	float m_auxAnalogAxis[MAX_VR_ANALOG_AXIS*2];
	int m_buttons[MAX_VR_BUTTONS];
};

typedef struct b3VREventsData
{
	int m_numControllerEvents;
	struct b3VRControllerEvent* m_controllerEvents;
	
};


typedef struct b3KeyboardEvent
{
	int m_keyCode;
	int m_keyState;
};

typedef struct b3KeyboardEventsData
{
	int m_numKeyboardEvents;
	struct b3KeyboardEvent* m_keyboardEvents;
};


enum eMouseEventTypeEnums
{
	MOUSE_MOVE_EVENT=1,
	MOUSE_BUTTON_EVENT=2,
};

typedef struct b3MouseEvent
{
	int m_eventType;
	float m_mousePosX;
	float m_mousePosY;
	int m_buttonIndex;
	int m_buttonState;
};

typedef struct b3MouseEventsData
{
	int m_numMouseEvents;
	struct b3MouseEvent* m_mouseEvents;
};

enum b3NotificationType {
	SIMULATION_RESET = 0,
	BODY_ADDED = 1,
	BODY_REMOVED = 2,
	USER_DATA_ADDED = 3,
	USER_DATA_REMOVED = 4,
	LINK_DYNAMICS_CHANGED = 5,
	VISUAL_SHAPE_CHANGED = 6,
	TRANSFORM_CHANGED = 7,
	SIMULATION_STEPPED = 8,
};

typedef struct b3BodyNotificationArgs
{
	int m_bodyUniqueId;
};

typedef struct b3UserDataNotificationArgs
{
	int m_userDataId;
};

typedef struct b3LinkNotificationArgs
{
	int m_bodyUniqueId;
	int m_linkIndex;
};

typedef struct b3VisualShapeNotificationArgs
{
	int m_bodyUniqueId;
	int m_linkIndex;
	int m_visualShapeIndex;
};

typedef struct b3TransformChangeNotificationArgs
{
	int m_bodyUniqueId;
	int m_linkIndex;
	double m_worldPosition[3];
	double m_worldRotation[4];
	double m_localScaling[3];
};

typedef struct b3Notification
{
	int m_notificationType;
	union {
		struct b3BodyNotificationArgs m_bodyArgs;
		struct b3UserDataNotificationArgs m_userDataArgs;
		struct b3LinkNotificationArgs m_linkArgs;
		struct b3VisualShapeNotificationArgs m_visualShapeArgs;
		struct b3TransformChangeNotificationArgs m_transformChangeArgs;
	};
};

typedef struct b3ContactPointData
{
    int m_contactFlags;
    int m_bodyUniqueIdA;
    int m_bodyUniqueIdB;
    int m_linkIndexA;
    int m_linkIndexB;
    double m_positionOnAInWS[3];
    double m_positionOnBInWS[3];
    double m_contactNormalOnBInWS[3];
    double m_contactDistance;
    
    double m_normalForce;
};

enum 
{
	CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS = 0,
	CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS = 1,
};

enum  b3StateLoggingType
{
	STATE_LOGGING_MINITAUR = 0,
	STATE_LOGGING_GENERIC_ROBOT = 1,
	STATE_LOGGING_VR_CONTROLLERS = 2,
	STATE_LOGGING_VIDEO_MP4 = 3,
	STATE_LOGGING_COMMANDS = 4,
	STATE_LOGGING_CONTACT_POINTS = 5,
	STATE_LOGGING_PROFILE_TIMINGS = 6,
	STATE_LOGGING_ALL_COMMANDS=7,
	STATE_REPLAY_ALL_COMMANDS=8,
	STATE_LOGGING_CUSTOM_TIMER=9,
};


typedef struct b3ContactInformation
{
	int m_numContactPoints;
	struct b3ContactPointData* m_contactPointData;
};

typedef struct b3RayData
{
  double m_rayFromPosition[3];
  double m_rayToPosition[3];
};

typedef struct b3RayHitInfo
{
	double m_hitFraction;
	int m_hitObjectUniqueId;
	int m_hitObjectLinkIndex;
	double m_hitPositionWorld[3];
	double m_hitNormalWorld[3];
};

typedef struct b3RaycastInformation
{
	int m_numRayHits;
	struct b3RayHitInfo* m_rayHits;
};

typedef union {
    struct b3RayData a;
    struct b3RayHitInfo b;
} RAY_DATA_UNION;


static const int MAX_RAY_INTERSECTION_BATCH_SIZE = 256;
static const int MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING = (16*1024);

static const int MAX_RAY_HITS = MAX_RAY_INTERSECTION_BATCH_SIZE;
static const int VISUAL_SHAPE_MAX_PATH_LEN = 1024;

enum b3VisualShapeDataFlags
{
	eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = 1,
};

typedef struct b3VisualShapeData
{
	int m_objectUniqueId;
	int m_linkIndex;
	int m_visualGeometryType;
	double m_dimensions[3];
	char m_meshAssetFileName[VISUAL_SHAPE_MAX_PATH_LEN];
    double m_localVisualFrame[7];
    double m_rgbaColor[4];
    int m_tinyRendererTextureId;
    int m_textureUniqueId;
    int m_openglTextureId;
    
};

typedef struct b3VisualShapeInformation
{
	int m_numVisualShapes;
	struct b3VisualShapeData* m_visualShapeData;
};


typedef struct b3CollisionShapeData
{
	int m_objectUniqueId;
	int m_linkIndex;
	int m_collisionGeometryType;
	double m_dimensions[3];
	double m_localCollisionFrame[7];
	char m_meshAssetFileName[VISUAL_SHAPE_MAX_PATH_LEN];
};

typedef struct b3CollisionShapeInformation
{
	int m_numCollisionShapes;
	struct b3CollisionShapeData* m_collisionShapeData;
};

enum eLinkStateFlags
{
	ACTUAL_STATE_COMPUTE_LINKVELOCITY=1,
	ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS=2,
};

///b3LinkState provides extra information such as the Cartesian world coordinates
///center of mass (COM) of the link, relative to the world reference frame.
///Orientation is a quaternion x,y,z,w
///Note: to compute the URDF link frame (which equals the joint frame at joint position 0)
///use URDF link frame = link COM frame * inertiaFrame.inverse()
typedef struct b3LinkState
{
    double m_worldPosition[3];
    double m_worldOrientation[4];

    double m_localInertialPosition[3];
    double m_localInertialOrientation[4];

	double m_worldLinkFramePosition[3];
    double m_worldLinkFrameOrientation[4];

	double m_worldLinearVelocity[3]; 
	double m_worldAngularVelocity[3]; 

	double m_worldAABBMin[3];
	double m_worldAABBMax[3];
};

//todo: discuss and decide about control mode and combinations
enum {
    CONTROL_MODE_VELOCITY=0,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_POSITION_VELOCITY_PD,
    CONTROL_MODE_PD, 
};

///flags for b3ApplyExternalTorque and b3ApplyExternalForce
enum EnumExternalForceFlags
{
    EF_LINK_FRAME=1,
    EF_WORLD_FRAME=2,
};

///flags to pick the renderer for synthetic camera
enum EnumRenderer
{
    ER_TINY_RENDERER=(1<<16),
    ER_BULLET_HARDWARE_OPENGL=(1<<17),
};

enum EnumRendererAuxFlags
{
	ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX=1,
	ER_USE_PROJECTIVE_TEXTURE=2,
};

///flags to pick the IK solver and other options
enum EnumCalculateInverseKinematicsFlags
{
	IK_DLS=0,
	IK_SDLS=1, 
	IK_HAS_TARGET_POSITION=16,
	IK_HAS_TARGET_ORIENTATION=32,
	IK_HAS_NULL_SPACE_VELOCITY=64,
	IK_HAS_JOINT_DAMPING=128,
	IK_HAS_CURRENT_JOINT_POSITIONS=256,
	IK_HAS_MAX_ITERATIONS=512,
	IK_HAS_RESIDUAL_THRESHOLD = 1024,
};

enum b3ConfigureDebugVisualizerEnum
{
    COV_ENABLE_GUI=1,
    COV_ENABLE_SHADOWS,
    COV_ENABLE_WIREFRAME,
	COV_ENABLE_VR_TELEPORTING,
	COV_ENABLE_VR_PICKING,
	COV_ENABLE_VR_RENDER_CONTROLLERS,
	COV_ENABLE_RENDERING,
	COV_ENABLE_SYNC_RENDERING_INTERNAL,
	COV_ENABLE_KEYBOARD_SHORTCUTS,
	COV_ENABLE_MOUSE_PICKING,
	COV_ENABLE_Y_AXIS_UP,
	COV_ENABLE_TINY_RENDERER,
	COV_ENABLE_RGB_BUFFER_PREVIEW,
	COV_ENABLE_DEPTH_BUFFER_PREVIEW,
	COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
	COV_ENABLE_PLANAR_REFLECTION,
	
};

enum b3AddUserDebugItemEnum
{
	DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA=1,
	DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2,
	DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4,
};

enum eCONNECT_METHOD {
  eCONNECT_GUI = 1,
  eCONNECT_DIRECT = 2,
  eCONNECT_SHARED_MEMORY = 3,
  eCONNECT_UDP = 4,
  eCONNECT_TCP = 5,
  eCONNECT_EXISTING_EXAMPLE_BROWSER=6,
  eCONNECT_GUI_SERVER=7,
  eCONNECT_GUI_MAIN_THREAD=8,
  eCONNECT_SHARED_MEMORY_SERVER=9,
  eCONNECT_DART=10,
  eCONNECT_MUJOCO=11,
};

enum eURDF_Flags
{
	URDF_USE_INERTIA_FROM_FILE=2,
	URDF_USE_SELF_COLLISION=8,
	URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16,
	URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32,
	URDF_RESERVED=64,
	URDF_USE_IMPLICIT_CYLINDER =128,
	URDF_GLOBAL_VELOCITIES_MB =256,
	MJCF_COLORS_FROM_FILE=512,
	URDF_ENABLE_CACHED_GRAPHICS_SHAPES=1024,
    URDF_ENABLE_SLEEPING=2048,
	URDF_INITIALIZE_SAT_FEATURES = 4096,
};

enum eUrdfGeomTypes 
{
	GEOM_SPHERE=2,
	GEOM_BOX,
	GEOM_CYLINDER,
	GEOM_MESH,
	GEOM_PLANE,
	GEOM_CAPSULE, 
	GEOM_UNKNOWN, 
};

enum eUrdfCollisionFlags
{
	GEOM_FORCE_CONCAVE_TRIMESH=1,
	GEOM_CONCAVE_INTERNAL_EDGE=2,
};

enum eUrdfVisualFlags
{
	GEOM_VISUAL_HAS_RGBA_COLOR=1,
	GEOM_VISUAL_HAS_SPECULAR_COLOR=2,
};


enum eStateLoggingFlags
{
	STATE_LOG_JOINT_MOTOR_TORQUES=1,
	STATE_LOG_JOINT_USER_TORQUES=2,
	STATE_LOG_JOINT_TORQUES = STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES,
};

enum eJointFeedbackModes
{
	JOINT_FEEDBACK_IN_WORLD_SPACE=1,
	JOINT_FEEDBACK_IN_JOINT_FRAME=2,
};

static const int B3_MAX_PLUGIN_ARG_SIZE = 128;
static const int B3_MAX_PLUGIN_ARG_TEXT_LEN = 1024;

typedef struct b3PluginArguments
{
	char m_text[B3_MAX_PLUGIN_ARG_TEXT_LEN];
	int m_numInts;
	int m_ints[B3_MAX_PLUGIN_ARG_SIZE];
	int m_numFloats;
	double m_floats[B3_MAX_PLUGIN_ARG_SIZE];
};

typedef struct b3PhysicsSimulationParameters
{
	double m_deltaTime;
	double m_gravityAcceleration[3];
	int m_numSimulationSubSteps;
	int m_numSolverIterations;
	int m_useRealTimeSimulation;
	int m_useSplitImpulse;
	double m_splitImpulsePenetrationThreshold;
	double m_contactBreakingThreshold;
	int m_internalSimFlags;
	double m_defaultContactERP;
	int m_collisionFilterMode;
	int m_enableFileCaching;
	double m_restitutionVelocityThreshold;
	double m_defaultNonContactERP;
	double m_frictionERP;
	double m_defaultGlobalCFM;
	double m_frictionCFM;
	int m_enableConeFriction;
	int m_deterministicOverlappingPairs;
	double m_allowedCcdPenetration;
	int m_jointFeedbackMode;
	double m_solverResidualThreshold;
	double m_contactSlop;
	int m_enableSAT;
};


/// ----------------------------------------------------------------------------------------------------------
typedef float	plReal;

typedef plReal	plVector3[3];
typedef plReal	plQuaternion[4];

//**	Bullet3 physics SDK (C-API) */
typedef struct b3PhysicsClientHandle__ { int unused; } *b3PhysicsClientHandle;

typedef struct b3SharedMemoryCommandHandle__ { int unused; } *b3SharedMemoryCommandHandle;
typedef struct b3SharedMemoryStatusHandle__ { int unused; } *b3SharedMemoryStatusHandle;

//**
//Create and Delete a Physics SDK
//*/

b3PhysicsClientHandle b3ConnectSharedMemory(int key);
b3PhysicsClientHandle b3ConnectSharedMemory2(int key);


///b3DisconnectSharedMemory will disconnect the client from the server and cleanup memory.
	void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

///There can only be 1 outstanding command. Check if a command can be send.
	int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

///blocking submit command and wait for status
	b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///In general it is better to use b3SubmitClientCommandAndWaitStatus. b3SubmitClientCommand is a non-blocking submit
///command, which requires checking for the status manually, using b3ProcessServerStatus. Also, before sending the
///next command, make sure to check if you can send a command using 'b3CanSubmitCommand'.
	int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///non-blocking check status
	b3SharedMemoryStatusHandle	b3ProcessServerStatus(b3PhysicsClientHandle physClient);

/// Get the physics server return status type. See EnumSharedMemoryServerStatus in SharedMemoryPublic.h for error codes.
	int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle);

///Plugin system, load and unload a plugin, execute a command
 b3SharedMemoryCommandHandle b3CreateCustomCommand(b3PhysicsClientHandle physClient);
	void b3CustomCommandLoadPlugin(b3SharedMemoryCommandHandle commandHandle, const char* pluginPath);
	void b3CustomCommandLoadPluginSetPostFix(b3SharedMemoryCommandHandle commandHandle, const char* postFix);
 int b3GetStatusPluginUniqueId(b3SharedMemoryStatusHandle statusHandle);
 int b3GetStatusPluginCommandResult(b3SharedMemoryStatusHandle statusHandle);

	void b3CustomCommandUnloadPlugin(b3SharedMemoryCommandHandle commandHandle, int pluginUniqueId);
	void b3CustomCommandExecutePluginCommand(b3SharedMemoryCommandHandle commandHandle, int pluginUniqueId, const char* textArguments);
	void b3CustomCommandExecuteAddIntArgument(b3SharedMemoryCommandHandle commandHandle, int intVal);
	void b3CustomCommandExecuteAddFloatArgument(b3SharedMemoryCommandHandle commandHandle, float floatVal);


	int b3GetStatusBodyIndices(b3SharedMemoryStatusHandle statusHandle, int* bodyIndicesOut, int bodyIndicesCapacity);

	int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle);

	int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
                           int* bodyUniqueId,
                           int* numDegreeOfFreedomQ,
                           int* numDegreeOfFreedomU,
                           const double* rootLocalInertialFrame[],
                           const double* actualStateQ[],
                           const double* actualStateQdot[],
                           const double* jointReactionForces[]);


	b3SharedMemoryCommandHandle b3RequestCollisionInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	int b3GetStatusAABB(b3SharedMemoryStatusHandle statusHandle, int linkIndex, double aabbMin[/*3*/], double aabbMax[/*3*/]);

///If you re-connected to an existing server, or server changed otherwise, sync the body info and user constraints etc.
	b3SharedMemoryCommandHandle b3InitSyncBodyInfoCommand(b3PhysicsClientHandle physClient);

	b3SharedMemoryCommandHandle b3InitRemoveBodyCommand(b3PhysicsClientHandle physClient, int bodyUniqueId);

///return the total number of bodies in the simulation
	int	b3GetNumBodies(b3PhysicsClientHandle physClient);

/// return the body unique id, given the index in range [0 , b3GetNumBodies() )
	int b3GetBodyUniqueId(b3PhysicsClientHandle physClient, int serialIndex);

///given a body unique id, return the body information. See b3BodyInfo in SharedMemoryPublic.h
	int b3GetBodyInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, struct b3BodyInfo* info);

///give a unique body index (after loading the body) return the number of joints.
	int	b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyUniqueId);

///compute the number of degrees of freedom for this body.
///Return -1 for unsupported spherical joint, -2 for unsupported planar joint.
	int	b3ComputeDofCount(b3PhysicsClientHandle physClient, int bodyUniqueId);

///given a body and joint index, return the joint information. See b3JointInfo in SharedMemoryPublic.h
	int	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, struct b3JointInfo* info);

///user data handling
 b3SharedMemoryCommandHandle b3InitSyncUserDataCommand(b3PhysicsClientHandle physClient);
 b3SharedMemoryCommandHandle b3InitAddUserDataCommand(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, const char* key, enum UserDataValueType valueType, int valueLength, const void *valueData);
 b3SharedMemoryCommandHandle b3InitRemoveUserDataCommand(b3PhysicsClientHandle physClient, int userDataId);

 int b3GetUserData(b3PhysicsClientHandle physClient, int userDataId, struct b3UserDataValue *valueOut);
 int b3GetUserDataId(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, const char *key);
 int b3GetUserDataIdFromStatus(b3SharedMemoryStatusHandle statusHandle);
 int b3GetNumUserData(b3PhysicsClientHandle physClient, int bodyUniqueId);
 void b3GetUserDataInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, int userDataIndex, const char **keyOut, int *userDataIdOut, int *linkIndexOut, int *visualShapeIndexOut);

	b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
///given a body unique id and link index, return the dynamics information. See b3DynamicsInfo in SharedMemoryPublic.h
	int b3GetDynamicsInfo(b3SharedMemoryStatusHandle statusHandle, struct b3DynamicsInfo* info);
	
	b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo(b3PhysicsClientHandle physClient);
	int b3ChangeDynamicsInfoSetMass(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double mass);
	int b3ChangeDynamicsInfoSetLocalInertiaDiagonal(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double localInertiaDiagonal[]);

	int b3ChangeDynamicsInfoSetLateralFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction);
	int b3ChangeDynamicsInfoSetSpinningFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
	int b3ChangeDynamicsInfoSetRollingFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
	int b3ChangeDynamicsInfoSetRestitution(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double restitution);
	int b3ChangeDynamicsInfoSetLinearDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double linearDamping);
	int b3ChangeDynamicsInfoSetAngularDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double angularDamping);
	int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex,double contactStiffness, double contactDamping);
	int b3ChangeDynamicsInfoSetFrictionAnchor(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex, int frictionAnchor);
	int b3ChangeDynamicsInfoSetCcdSweptSphereRadius(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex, double ccdSweptSphereRadius);
	int b3ChangeDynamicsInfoSetContactProcessingThreshold(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double contactProcessingThreshold);
	int b3ChangeDynamicsInfoSetActivationState(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int activationState);

	b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand(b3PhysicsClientHandle physClient, int parentBodyUniqueId, int parentJointIndex, int childBodyUniqueId, int childJointIndex, struct b3JointInfo* info);

///return a unique id for the user constraint, after successful creation, or -1 for an invalid constraint id
	int b3GetStatusUserConstraintUniqueId(b3SharedMemoryStatusHandle statusHandle);

///change parameters of an existing user constraint
	b3SharedMemoryCommandHandle  b3InitChangeUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);
	int b3InitChangeUserConstraintSetPivotInB(b3SharedMemoryCommandHandle commandHandle, double jointChildPivot[/*3*/]);
	int b3InitChangeUserConstraintSetFrameInB(b3SharedMemoryCommandHandle commandHandle, double jointChildFrameOrn[/*4*/]);
	int b3InitChangeUserConstraintSetMaxForce(b3SharedMemoryCommandHandle commandHandle, double maxAppliedForce);
	int b3InitChangeUserConstraintSetGearRatio(b3SharedMemoryCommandHandle commandHandle, double gearRatio);
	int b3InitChangeUserConstraintSetGearAuxLink(b3SharedMemoryCommandHandle commandHandle, int gearAuxLink);
	int b3InitChangeUserConstraintSetRelativePositionTarget(b3SharedMemoryCommandHandle commandHandle, double relativePositionTarget);
	int b3InitChangeUserConstraintSetERP(b3SharedMemoryCommandHandle commandHandle, double erp);

	b3SharedMemoryCommandHandle  b3InitRemoveUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);

	int b3GetNumUserConstraints(b3PhysicsClientHandle physClient);

	b3SharedMemoryCommandHandle b3InitGetUserConstraintStateCommand(b3PhysicsClientHandle physClient, int constraintUniqueId);
	int b3GetStatusUserConstraintState(b3SharedMemoryStatusHandle statusHandle, struct b3UserConstraintState* constraintState);


	int b3GetUserConstraintInfo(b3PhysicsClientHandle physClient, int constraintUniqueId, struct b3UserConstraint* info);
/// return the user constraint id, given the index in range [0 , b3GetNumUserConstraints() )
	int b3GetUserConstraintId(b3PhysicsClientHandle physClient, int serialIndex);
    
///Request physics debug lines for debug visualization. The flags in debugMode are the same as used in Bullet
///See btIDebugDraw::DebugDrawModes in Bullet/src/LinearMath/btIDebugDraw.h
	b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);

///Get the pointers to the physics debug line information, after b3InitRequestDebugLinesCommand returns
///status CMD_DEBUG_LINES_COMPLETED
	void    b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);
    
///configure the 3D OpenGL debug visualizer (enable/disable GUI widgets, shadows, position camera etc)
	b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer(b3PhysicsClientHandle physClient);
	void b3ConfigureOpenGLVisualizerSetVisualizationFlags(b3SharedMemoryCommandHandle commandHandle, int flag, int enabled);
	void b3ConfigureOpenGLVisualizerSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, const float cameraTargetPosition[/*3*/]);

	b3SharedMemoryCommandHandle b3InitRequestOpenGLVisualizerCameraCommand(b3PhysicsClientHandle physClient);
	int b3GetStatusOpenGLVisualizerCamera(b3SharedMemoryStatusHandle statusHandle, struct b3OpenGLVisualizerCameraInfo* camera);

    
/// Add/remove user-specific debug lines and debug text messages
	b3SharedMemoryCommandHandle b3InitUserDebugDrawAddLine3D(b3PhysicsClientHandle physClient, double fromXYZ[/*3*/], double toXYZ[/*3*/], double colorRGB[/*3*/], double lineWidth, double lifeTime);

	b3SharedMemoryCommandHandle b3InitUserDebugDrawAddText3D(b3PhysicsClientHandle physClient, const char* txt, double positionXYZ[/*3*/], double colorRGB[/*3*/], double textSize, double lifeTime);
	void b3UserDebugTextSetOptionFlags(b3SharedMemoryCommandHandle commandHandle, int optionFlags);
	void b3UserDebugTextSetOrientation(b3SharedMemoryCommandHandle commandHandle, double orientation[/*4*/]);
	void b3UserDebugItemSetReplaceItemUniqueId(b3SharedMemoryCommandHandle commandHandle, int replaceItem);


	void b3UserDebugItemSetParentObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

	b3SharedMemoryCommandHandle b3InitUserDebugAddParameter(b3PhysicsClientHandle physClient, const char* txt, double rangeMin, double rangeMax, double startValue);
	b3SharedMemoryCommandHandle b3InitUserDebugReadParameter(b3PhysicsClientHandle physClient, int debugItemUniqueId);
	int b3GetStatusDebugParameterValue(b3SharedMemoryStatusHandle statusHandle, double* paramValue);

	b3SharedMemoryCommandHandle b3InitUserDebugDrawRemove(b3PhysicsClientHandle physClient, int debugItemUniqueId);
	b3SharedMemoryCommandHandle b3InitUserDebugDrawRemoveAll(b3PhysicsClientHandle physClient);

	b3SharedMemoryCommandHandle b3InitDebugDrawingCommand(b3PhysicsClientHandle physClient);
	void b3SetDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex, double objectColorRGB[/*3*/]);
	void b3RemoveDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

///All debug items unique Ids are positive: a negative unique Id means failure.
	int b3GetDebugItemUniqueId(b3SharedMemoryStatusHandle statusHandle);


///request an image from a simulated camera, using a software renderer.
	b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient);
	void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle commandHandle, float viewMatrix[/*16*/], float projectionMatrix[/*16*/]);
	void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle commandHandle, int width, int height );
	void b3RequestCameraImageSetLightDirection(b3SharedMemoryCommandHandle commandHandle, const float lightDirection[/*3*/]);
	void b3RequestCameraImageSetLightColor(b3SharedMemoryCommandHandle commandHandle, const float lightColor[/*3*/]);
	void b3RequestCameraImageSetLightDistance(b3SharedMemoryCommandHandle commandHandle, float lightDistance);
	void b3RequestCameraImageSetLightAmbientCoeff(b3SharedMemoryCommandHandle commandHandle, float lightAmbientCoeff);
	void b3RequestCameraImageSetLightDiffuseCoeff(b3SharedMemoryCommandHandle commandHandle, float lightDiffuseCoeff);
	void b3RequestCameraImageSetLightSpecularCoeff(b3SharedMemoryCommandHandle commandHandle, float lightSpecularCoeff);
	void b3RequestCameraImageSetShadow(b3SharedMemoryCommandHandle commandHandle, int hasShadow);
	void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer);
	void b3RequestCameraImageSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


	void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData);
	
///set projective texture camera matrices.
	void b3RequestCameraImageSetProjectiveTextureMatrices(b3SharedMemoryCommandHandle commandHandle, float viewMatrix[/*16*/], float projectionMatrix[/*16*/]);

///compute a view matrix, helper function for b3RequestCameraImageSetCameraMatrices
	void b3ComputeViewMatrixFromPositions(const float cameraPosition[/*3*/], const float cameraTargetPosition[/*3*/], const float cameraUp[/*3*/], float viewMatrix[/*16*/]);
	void b3ComputeViewMatrixFromYawPitchRoll(const float cameraTargetPosition[/*3*/], float distance, float yaw, float pitch, float roll, int upAxis, float viewMatrix[/*16*/]);
	void b3ComputePositionFromViewMatrix(const float viewMatrix[/*16*/], float cameraPosition[/*3*/], float cameraTargetPosition[/*3*/], float cameraUp[/*3*/]);

///compute a projection matrix, helper function for b3RequestCameraImageSetCameraMatrices
	void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, float projectionMatrix[/*16*/]);
	void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float projectionMatrix[/*16*/]);


/* obsolete, please use b3ComputeViewProjectionMatrices */
	void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, const float cameraPosition[/*3*/], const float cameraTargetPosition[/*3*/], const float cameraUp[/*3*/]);
/* obsolete, please use b3ComputeViewProjectionMatrices */
	void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[/*3*/], float distance, float yaw, float pitch, float roll, int upAxis);
/* obsolete, please use b3ComputeViewProjectionMatrices */
	void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float left, float right, float bottom, float top, float nearVal, float farVal);
/* obsolete, please use b3ComputeViewProjectionMatrices */
	void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float fov, float aspect, float nearVal, float farVal);


///request an contact point information
	b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient);
	void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
	void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
	void b3SetContactFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	void b3SetContactFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointData);

///compute the closest points between two bodies
	b3SharedMemoryCommandHandle b3InitClosestDistanceQuery(b3PhysicsClientHandle physClient);
	void b3SetClosestDistanceFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
	void b3SetClosestDistanceFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	void b3SetClosestDistanceFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
	void b3SetClosestDistanceFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	void b3SetClosestDistanceThreshold(b3SharedMemoryCommandHandle commandHandle, double distance);
	void b3GetClosestPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);

///get all the bodies that touch a given axis aligned bounding box specified in world space (min and max coordinates)
	b3SharedMemoryCommandHandle b3InitAABBOverlapQuery(b3PhysicsClientHandle physClient, const double aabbMin[/*3*/],const double aabbMax[/*3*/]);
	void b3GetAABBOverlapResults(b3PhysicsClientHandle physClient, struct b3AABBOverlapData* data);

//request visual shape information
	b3SharedMemoryCommandHandle b3InitRequestVisualShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueIdA);
	void b3GetVisualShapeInformation(b3PhysicsClientHandle physClient, struct b3VisualShapeInformation* visualShapeInfo);

	b3SharedMemoryCommandHandle b3InitRequestCollisionShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
	void b3GetCollisionShapeInformation(b3PhysicsClientHandle physClient, struct b3CollisionShapeInformation* collisionShapeInfo);



	b3SharedMemoryCommandHandle b3InitLoadTexture(b3PhysicsClientHandle physClient, const char* filename);
	int b3GetStatusTextureUniqueId(b3SharedMemoryStatusHandle statusHandle);

	b3SharedMemoryCommandHandle b3CreateChangeTextureCommandInit(b3PhysicsClientHandle physClient, int textureUniqueId, int width, int height, const char* rgbPixels);

	b3SharedMemoryCommandHandle b3InitUpdateVisualShape(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);
	void b3UpdateVisualShapeRGBAColor(b3SharedMemoryCommandHandle commandHandle, double rgbaColor[/*4*/]);
	void b3UpdateVisualShapeSpecularColor(b3SharedMemoryCommandHandle commandHandle, double specularColor[/*3*/]);


	b3SharedMemoryCommandHandle	b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
	int	b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz);
	int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
	int	b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
	int	b3PhysicsParamSetDefaultNonContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultNonContactERP);
	int	b3PhysicsParamSetDefaultFrictionERP(b3SharedMemoryCommandHandle commandHandle, double frictionERP);
 int b3PhysicsParamSetDefaultGlobalCFM(b3SharedMemoryCommandHandle commandHandle, double defaultGlobalCFM);
 int b3PhysicsParamSetDefaultFrictionCFM(b3SharedMemoryCommandHandle commandHandle, double frictionCFM);
	int	b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
	int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
	int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);
	int b3PhysicsParamSetCollisionFilterMode(b3SharedMemoryCommandHandle commandHandle, int filterMode);
	int b3PhysicsParamSetUseSplitImpulse(b3SharedMemoryCommandHandle commandHandle, int useSplitImpulse);
	int b3PhysicsParamSetSplitImpulsePenetrationThreshold(b3SharedMemoryCommandHandle commandHandle, double splitImpulsePenetrationThreshold);
	int b3PhysicsParamSetContactBreakingThreshold(b3SharedMemoryCommandHandle commandHandle, double contactBreakingThreshold);
	int b3PhysicsParamSetMaxNumCommandsPer1ms(b3SharedMemoryCommandHandle commandHandle, int maxNumCmdPer1ms);
	int b3PhysicsParamSetEnableFileCaching(b3SharedMemoryCommandHandle commandHandle, int enableFileCaching);
	int b3PhysicsParamSetRestitutionVelocityThreshold(b3SharedMemoryCommandHandle commandHandle, double restitutionVelocityThreshold);
	int b3PhysicsParamSetEnableConeFriction(b3SharedMemoryCommandHandle commandHandle, int enableConeFriction);
	int b3PhysicsParameterSetDeterministicOverlappingPairs(b3SharedMemoryCommandHandle commandHandle, int deterministicOverlappingPairs);
	int b3PhysicsParameterSetAllowedCcdPenetration(b3SharedMemoryCommandHandle commandHandle, double allowedCcdPenetration);
	int b3PhysicsParameterSetJointFeedbackMode(b3SharedMemoryCommandHandle commandHandle, int jointFeedbackMode);
	int b3PhysicsParamSetSolverResidualThreshold(b3SharedMemoryCommandHandle commandHandle, double solverResidualThreshold);
	int b3PhysicsParamSetContactSlop(b3SharedMemoryCommandHandle commandHandle, double contactSlop);
	int b3PhysicsParameterSetEnableSAT(b3SharedMemoryCommandHandle commandHandle, int enableSAT);






	b3SharedMemoryCommandHandle	b3InitRequestPhysicsParamCommand(b3PhysicsClientHandle physClient);
	int b3GetStatusPhysicsSimulationParameters(b3SharedMemoryStatusHandle statusHandle,struct b3PhysicsSimulationParameters* params);


//b3PhysicsParamSetInternalSimFlags is for internal/temporary/easter-egg/experimental demo purposes
//Use at own risk: magic things may or my not happen when calling this API
	int b3PhysicsParamSetInternalSimFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


 b3SharedMemoryCommandHandle	b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);

 b3SharedMemoryCommandHandle	b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);

///Load a robot from a URDF file. Status type will CMD_URDF_LOADING_COMPLETED.
///Access the robot from the unique body index, through b3GetStatusBodyIndex(statusHandle);
 b3SharedMemoryCommandHandle	b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);

 int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
 int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
 int	b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
 int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);
 int	b3LoadUrdfCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);
 int	b3LoadUrdfCommandSetGlobalScaling(b3SharedMemoryCommandHandle commandHandle, double globalScaling);




 b3SharedMemoryCommandHandle b3SaveStateCommandInit(b3PhysicsClientHandle physClient);
 int b3GetStatusGetStateId(b3SharedMemoryStatusHandle statusHandle);

	b3SharedMemoryCommandHandle	b3LoadStateCommandInit(b3PhysicsClientHandle physClient);
 int	b3LoadStateSetStateId(b3SharedMemoryCommandHandle commandHandle, int stateId);
 int	b3LoadStateSetFileName(b3SharedMemoryCommandHandle commandHandle, const char* fileName);

	b3SharedMemoryCommandHandle	b3LoadBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);

	b3SharedMemoryCommandHandle	b3SaveBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	b3SharedMemoryCommandHandle	b3LoadMJCFCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	void b3LoadMJCFCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


///compute the forces to achieve an acceleration, given a state q and qdot using inverse dynamics
	b3SharedMemoryCommandHandle	b3CalculateInverseDynamicsCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId,
	const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
	int b3GetStatusInverseDynamicsJointForces(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointForces);

	b3SharedMemoryCommandHandle	b3CalculateJacobianCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
	int b3GetStatusJacobian(b3SharedMemoryStatusHandle statusHandle,
	int* dofCount,
	double* linearJacobian,
	double* angularJacobian);

	b3SharedMemoryCommandHandle	b3CalculateMassMatrixCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, const double* jointPositionsQ);
///the mass matrix is stored in column-major layout of size dofCount*dofCount
	int b3GetStatusMassMatrix(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle,	int* dofCount,	double* massMatrix);


///compute the joint positions to move the end effector to a desired target using inverse kinematics
	b3SharedMemoryCommandHandle	b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	void b3CalculateInverseKinematicsAddTargetPurePosition(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[/*3*/]);
	void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double targetOrientation[/*4*/]);
	void b3CalculateInverseKinematicsPosWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
	void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double targetOrientation[/*4*/], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
	void b3CalculateInverseKinematicsSetJointDamping(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* jointDampingCoeff);
	void b3CalculateInverseKinematicsSelectSolver(b3SharedMemoryCommandHandle commandHandle, int solver);
	int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointPositions);

	void b3CalculateInverseKinematicsSetCurrentPositions(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* currentJointPositions);
	void b3CalculateInverseKinematicsSetMaxNumIterations(b3SharedMemoryCommandHandle commandHandle, int maxNumIterations);
	void b3CalculateInverseKinematicsSetResidualThreshold(b3SharedMemoryCommandHandle commandHandle, double residualThreshold);


	b3SharedMemoryCommandHandle	b3LoadSdfCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);
	int	b3LoadSdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
	int	b3LoadSdfCommandSetUseGlobalScaling(b3SharedMemoryCommandHandle commandHandle, double globalScaling);



	b3SharedMemoryCommandHandle	b3SaveWorldCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);


///The b3JointControlCommandInit method is obsolete, use b3JointControlCommandInit2 instead
	b3SharedMemoryCommandHandle  b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);

    
///Set joint motor control variables such as desired position/angle, desired velocity,
///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
	b3SharedMemoryCommandHandle  b3JointControlCommandInit2(b3PhysicsClientHandle physClient, int bodyUniqueId, int controlMode);

///Only use when controlMode is CONTROL_MODE_POSITION_VELOCITY_PD
	int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
	int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
	int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
	int b3JointControlSetMaximumVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double maximumVelocity);


///Only use when controlMode is CONTROL_MODE_VELOCITY
	int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value); /* find a better name for dof/q/u indices, point to b3JointInfo */
	int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value);
///Only use if when controlMode is CONTROL_MODE_TORQUE,
	int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value);
    

///the creation of collision shapes and rigid bodies etc is likely going to change,
///but good to have a b3CreateBoxShapeCommandInit for now


	b3SharedMemoryCommandHandle b3CreateCollisionShapeCommandInit(b3PhysicsClientHandle physClient);
	int b3CreateCollisionShapeAddSphere(b3SharedMemoryCommandHandle commandHandle,double radius);
	int b3CreateCollisionShapeAddBox(b3SharedMemoryCommandHandle commandHandle,double halfExtents[/*3*/]);
	int b3CreateCollisionShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
	int b3CreateCollisionShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
	int b3CreateCollisionShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, double planeNormal[/*3*/], double planeConstant);
	int b3CreateCollisionShapeAddMesh(b3SharedMemoryCommandHandle commandHandle,const char* fileName, double meshScale[/*3*/]);
	void b3CreateCollisionSetFlag(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, int flags);
	void b3CreateCollisionShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double childPosition[/*3*/], double childOrientation[/*4*/]);
	int b3GetStatusCollisionShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

	b3SharedMemoryCommandHandle b3CreateVisualShapeCommandInit(b3PhysicsClientHandle physClient);
	int b3CreateVisualShapeAddSphere(b3SharedMemoryCommandHandle commandHandle,double radius);
	int b3CreateVisualShapeAddBox(b3SharedMemoryCommandHandle commandHandle,double halfExtents[/*3*/]);
	int b3CreateVisualShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
	int b3CreateVisualShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
	int b3CreateVisualShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, double planeNormal[/*3*/], double planeConstant);
	int b3CreateVisualShapeAddMesh(b3SharedMemoryCommandHandle commandHandle,const char* fileName, double meshScale[/*3*/]);
	void b3CreateVisualSetFlag(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, int flags);
	void b3CreateVisualShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double childPosition[/*3*/], double childOrientation[/*4*/]);
	void b3CreateVisualShapeSetSpecularColor(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double specularColor[/*3*/]);
	void b3CreateVisualShapeSetRGBAColor(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double rgbaColor[/*4*/]);

	int b3GetStatusVisualShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

	b3SharedMemoryCommandHandle b3CreateMultiBodyCommandInit(b3PhysicsClientHandle physClient);
	int b3CreateMultiBodyBase(b3SharedMemoryCommandHandle commandHandle, 
		double mass, int collisionShapeUnique, int visualShapeUniqueId, 
		double basePosition[/*3*/], double baseOrientation[/*4*/] , 
		double baseInertialFramePosition[/*3*/], 
		double baseInertialFrameOrientation[/*4*/]);

	int b3CreateMultiBodyLink(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex, 
									double linkVisualShapeIndex, 
									double linkPosition[/*3*/], 
									double linkOrientation[/*4*/],
									double linkInertialFramePosition[/*3*/],
									double linkInertialFrameOrientation[/*4*/],
									int linkParentIndex,
									int linkJointType,
									double linkJointAxis[/*3*/]);


//useMaximalCoordinates are disabled by default, enabling them is experimental and not fully supported yet
	void b3CreateMultiBodyUseMaximalCoordinates(b3SharedMemoryCommandHandle commandHandle);
	void b3CreateMultiBodySetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


//int b3CreateMultiBodyAddLink(b3SharedMemoryCommandHandle commandHandle, int jointType, int parentLinkIndex, double linkMass, int linkCollisionShapeUnique, int linkVisualShapeUniqueId);



///create a box of size (1,1,1) at world origin (0,0,0) at orientation quat (0,0,0,1)
///after that, you can optionally adjust the initial position, orientation and size
	b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient);
	int	b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
	int	b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
	int	b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX,double halfExtentsY,double halfExtentsZ);
	int	b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
	int	b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType);
	int	b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red,double green,double blue, double alpha);


///b3CreatePoseCommandInit will initialize (teleport) the pose of a body/robot. You can individually set the base position,
///base orientation and joint angles. This will set all velocities of base and joints to zero.
///This is not a robot control command using actuators/joint motors, but manual repositioning the robot.
	b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	int	b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
	int	b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
	int	b3CreatePoseCommandSetBaseLinearVelocity(b3SharedMemoryCommandHandle commandHandle, double linVel[/*3*/]);
	int	b3CreatePoseCommandSetBaseAngularVelocity(b3SharedMemoryCommandHandle commandHandle, double angVel[/*3*/]);

	int	b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
	int	b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointPosition);

	int	b3CreatePoseCommandSetJointVelocities(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int numJointVelocities, const double* jointVelocities);
	int	b3CreatePoseCommandSetJointVelocity(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointVelocity);

///We are currently not reading the sensor information from the URDF file, and programmatically assign sensors.
///This is rather inconsistent, to mix programmatical creation with loading from file.
	b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
///b3CreateSensorEnableIMUForLink is not implemented yet.
///For now, if the IMU is located in the root link, use the root world transform to mimic an IMU.
	int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);

	b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient,int bodyUniqueId);
	int b3RequestActualStateCommandComputeLinkVelocity(b3SharedMemoryCommandHandle commandHandle, int computeLinkVelocity);
	int b3RequestActualStateCommandComputeForwardKinematics(b3SharedMemoryCommandHandle commandHandle, int computeForwardKinematics);


	int b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState *state);
	int b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, struct b3LinkState *state);

	b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
	b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ);
	b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);

	b3SharedMemoryCommandHandle b3CreateRaycastCommandInit(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);

	b3SharedMemoryCommandHandle b3CreateRaycastBatchCommandInit(b3PhysicsClientHandle physClient);
// Sets the number of threads to use to compute the ray intersections for the batch. Specify 0 to let Bullet decide, 1 (default) for single core execution, 2 or more to select the number of threads to use.
  void b3RaycastBatchSetNumThreads(b3SharedMemoryCommandHandle commandHandle, int numThreads);
//max num rays for b3RaycastBatchAddRay is MAX_RAY_INTERSECTION_BATCH_SIZE
	void b3RaycastBatchAddRay(b3SharedMemoryCommandHandle commandHandle, const double rayFromWorld[3], const double rayToWorld[3]);
//max num rays for b3RaycastBatchAddRays is MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING 
	void b3RaycastBatchAddRays(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double* rayFromWorld, const double* rayToWorld, int numRays);

	void b3GetRaycastInformation(b3PhysicsClientHandle physClient, struct b3RaycastInformation* raycastInfo);



/// Apply external force at the body (or link) center of mass, in world space/Cartesian coordinates.
	b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient);
	void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[/*3*/], const double position[/*3*/], int flag);
	void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[/*3*/], int flag);

///experiments of robots interacting with non-rigid objects (such as btSoftBody)
	b3SharedMemoryCommandHandle	b3LoadSoftBodyCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	int b3LoadSoftBodySetScale(b3SharedMemoryCommandHandle commandHandle, double scale);
	int b3LoadSoftBodySetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
	int b3LoadSoftBodySetCollisionMargin(b3SharedMemoryCommandHandle commandHandle, double collisionMargin);


	b3SharedMemoryCommandHandle	b3RequestVREventsCommandInit(b3PhysicsClientHandle physClient);
	void b3VREventsSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);

	void b3GetVREventsData(b3PhysicsClientHandle physClient, struct b3VREventsData* vrEventsData);

	b3SharedMemoryCommandHandle	b3SetVRCameraStateCommandInit(b3PhysicsClientHandle physClient);
	int b3SetVRCameraRootPosition(b3SharedMemoryCommandHandle commandHandle, double rootPos[/*3*/]);
	int b3SetVRCameraRootOrientation(b3SharedMemoryCommandHandle commandHandle, double rootOrn[/*4*/]);
	int b3SetVRCameraTrackingObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
	int b3SetVRCameraTrackingObjectFlag(b3SharedMemoryCommandHandle commandHandle, int flag);

	b3SharedMemoryCommandHandle	b3RequestKeyboardEventsCommandInit(b3PhysicsClientHandle physClient);
	void b3GetKeyboardEventsData(b3PhysicsClientHandle physClient, struct b3KeyboardEventsData* keyboardEventsData);

	b3SharedMemoryCommandHandle	b3RequestMouseEventsCommandInit(b3PhysicsClientHandle physClient);
	void b3GetMouseEventsData(b3PhysicsClientHandle physClient, struct b3MouseEventsData* mouseEventsData);


	b3SharedMemoryCommandHandle	b3StateLoggingCommandInit(b3PhysicsClientHandle physClient);
	int b3StateLoggingStart(b3SharedMemoryCommandHandle commandHandle, int loggingType, const char* fileName);
	int b3StateLoggingAddLoggingObjectUniqueId(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
	int b3StateLoggingSetMaxLogDof(b3SharedMemoryCommandHandle commandHandle, int maxLogDof);
	int b3StateLoggingSetLinkIndexA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	int b3StateLoggingSetLinkIndexB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	int b3StateLoggingSetBodyAUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyAUniqueId);
	int b3StateLoggingSetBodyBUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyBUniqueId);
	int b3StateLoggingSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);
	int b3StateLoggingSetLogFlags(b3SharedMemoryCommandHandle commandHandle, int logFlags);

	int b3GetStatusLoggingUniqueId(b3SharedMemoryStatusHandle statusHandle);
	int b3StateLoggingStop(b3SharedMemoryCommandHandle commandHandle, int loggingUid);


	b3SharedMemoryCommandHandle	b3ProfileTimingCommandInit(b3PhysicsClientHandle physClient, const char* name);
	void b3SetProfileTimingDuractionInMicroSeconds(b3SharedMemoryCommandHandle commandHandle, int duration);

	void b3PushProfileTiming(b3PhysicsClientHandle physClient, const char* timingName);
	void b3PopProfileTiming(b3PhysicsClientHandle physClient);

	void b3SetTimeOut(b3PhysicsClientHandle physClient, double timeOutInSeconds);
	double b3GetTimeOut(b3PhysicsClientHandle physClient);

	b3SharedMemoryCommandHandle b3SetAdditionalSearchPath(b3PhysicsClientHandle physClient, const char* path);

	void b3MultiplyTransforms(const double posA[/*3*/], const double ornA[/*4*/], const double posB[/*3*/], const double ornB[/*4*/], double outPos[/*3*/], double outOrn[/*4*/]);
	void b3InvertTransform(const double pos[/*3*/], const double orn[/*4*/], double outPos[/*3*/], double outOrn[/*4*/]);

    ]]

return bullet