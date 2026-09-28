/**
 * @file ty_rlm_func.h
 * @brief The ty_rlm_func module is used to register rlm complex functions.
 * @author guantc
 * @version 1.0
 * @date 2024-11-14 19:40:00
 *
 * Copyright (c) tuya.inc 2014-2021.
 *
 */

#ifndef __TY_RLM_FUNC_H__
#define __TY_RLM_FUNC_H__

#include "tuya_cloud_com_defs.h"
#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************macro define************************
***********************************************************/
#define RESERVE_VAL (-1)

#define RLM_SCHEDULE_PLAN_NAME_MAX 72   /* Max buffer size for schedule plan name (bytes) */
/***********************************************************
***********************typedef define***********************
***********************************************************/
/**
 * @brief lrm Complex function command set
 */
typedef enum {
    IMMEDIATE_MOWING_TASK_REQUEST = 0, // Mowing task
    IMMEDIATE_CHARGE_TASK_REQUEST,     // Charging task
    IMMEDIATE_PAUSE_TASK_REQUEST,      // Pause task
    DEV_REMOTE_CONTROL_TASK_REQUEST,   // Remote control task   
    DEV_SUBSCRIBE_REQUEST,             // Subscribe status type
    VECTOR_MAP_EDITING_REQUEST,        // Vector map - editing
    VECTOR_RECORD_ELEMENT_REQUEST,     // Vector map - record command       
    VECTOR_DELETE_ELEMENT_REQUEST,     // Vector map - delete vector element   
    VECTOR_UNDO_POINT_REQUEST,         // Vector map - undo record point 
    VECTOR_SAMPLING_REQUEST,           // Vector map - start auto sampling mode

    VECTOR_GET_MAP_INFO_REQUEST,       // Get map details
    VECTOR_GET_RECORD_INFO_REQUEST,    // Get mowing records
    VECTOR_GET_MAP_LIST_REQUEST,       // Get map list
    VECTOR_MAP_DELETE_REQUEST,         // Delete map

    MOW_RECORD_INFO_LIST_REQUEST,      // Get mowing record list
    ERROR_RECORD_INFO_LIST_REQUEST,    // Get error record list
    
    SCHEDULE_QUERY_REQUEST,                   // Calendar scheduling including get complete schedule list
    SCHEDULE_CREATE_REQUEST,                  // Create schedule
    SCHEDULE_UPDATE_REQUEST,                  // Update schedule
    SCHEDULE_DELETE_PERIOD_REQUEST,           // Delete specific time period (DELETE period)
    SCHEDULE_ENABLE_REQUEST,                  // Enable timer
    SCHEDULE_DISABLE_REQUEST,                 // Disable timer

    CONFIGURATION_GET_REQUEST,                // Robot get configuration   
    CONFIGURATION_SET_RAINY_DAY_MODE_REQUEST, // Robot set rainy day mode
    CONFIGURATION_SET_CUT_SWITCH_REQUEST,     // Robot set cutting switch
    CONFIGURATION_SET_SPEED_REQUEST,          // Robot set manual speed    
    CONFIGURATION_SET_BASE_REQUEST,           // Robot set base station
    CONFIGURATION_SET_PASSWORD_REQUEST,       // Robot set password
    TASK_SETTINGS_GET_REQUEST,                // Get lawn task settings
    TASK_SETTINGS_SET_REQUEST,                // Lawn task configuration
    PLANNED_PATH_GET_REQUEST,                 // Get planned path
    MOWING_PATH_GET_REQUEST,                  // Get mowing path
    CUSTOM_SHAPE_SET_REQUEST,                 // Set custom shape
    COMPLEX_RLM_CMD_MAX,
} COMPLEX_RLM_CMD_E;

// ============================================================================
// COMMON - Common definitions
// ============================================================================
/*Error status level (used for error information classification)*/
typedef enum {
    LEVEL_UNKNOWN = 0,
    LEVEL_INFO,     // General information level
    LEVEL_WARNING,  // Warning level
    LEVEL_ERROR,    // Exception information level
    LEVEL_CRITICAL, // Critical information level
} SEVERITY_LEVEL_E;

/*Day of week enumeration*/
typedef enum {
    ST_MONDAY = 0,
    ST_TUESDAY = 1,
    ST_WEDNESDAY = 2,
    ST_THURSDAY = 3,
    ST_FRIDAY = 4,
    ST_SATURDAY = 5,
    ST_SUNDAY = 6,
    ST_MAX_DAY,
} DAYOFWEEK_E;

/*Calendar operation type*/
typedef enum {
    TIMER_QUERY = 0, // Query calendar
    TIMER_CREATE,    // Create timer record
    TIMER_UPDATE,    // Update timer record
    TIMER_DELETE,    // Delete event
    TIMER_ENABLE,    // Enable time period
    TIMER_DISABLE,   // Disable time period
} CALENDAR_OPERATION_E;

/*Task status enumeration*/
typedef enum {
    // Standard status (0-99)
    TASK_STATUS_UNKNOWN = 0,    // Unknown status
    TASK_STATUS_STANDBY,        // Standby
    TASK_STATUS_CHARGING,       // Charging
    TASK_STATUS_MOWING,         // Mowing
    TASK_STATUS_PAUSED,         // Paused
    TASK_STATUS_RETURNING,      // Returning to station
    TASK_STATUS_UPDATING,       // Updating
    TASK_STATUS_SELF_CHECKING,  // Self-checking
    TASK_STATUS_MAPPING,        // Mapping
    // Custom status range (100-255)
    // 
    // Note: enum values are essentially integers that allow receiving 
    // integer values not defined in the enumeration
    // Therefore, devices can use any integer value in the 100-255 range as custom status
    // Even if these values are not defined in the enumeration, can still parse them successfully
    // 
    // Usage rules:
    // - Values in the 100-255 range are for custom use by the device side
    // - Devices can define the specific meaning of these statuses according to their needs
    // - It is recommended to document the meaning of each custom status value in device documentation
    // 
    // Example (documented only in device documentation, do not modify this file):
    //   101 = Device initializing
    //   102 = Waiting for user confirmation
    //   103 = Sensor calibrating
    //   104 = Network connecting
    //   ... etc.
    TASK_STATUS_CUSTOM_START = 100, // Mark custom status start value (100-255 range used by convention)
    TASK_STATUS_CUSTOM_END = 255,   // Mark custom status end value
} TASK_STATUS_E;


/**
 * @brief  End reason indicated by mowing record
 */
typedef enum {
    MOWING_RESULT_OK = 0,           // Robot ended normally
    MOWING_RESULT_MANUAL_STOP,      // Manually stopped
    MOWING_RESULT_LOW_BATTERY,      // Low battery
    MOWING_RESULT_RAIN_DETECTED,    // Rain detected
    MOWING_RESULT_OBSTACLE_BLOCKED, // Obstacle blocked
    MOWING_RESULT_SYSTEM_ERROR,     // System error    
    MOWING_RESULT_SCHEDULED_RETURN, // Scheduled return 
    MOWING_RESULT_MAX
} MOWING_RESULT_E;

// ============================================================================
// MAP - Map related definitions
// ============================================================================

/*Map element type*/
typedef enum {
    ELEMENT_UNKNOWN = 0,
    ELEMENT_BOUNDARY,      // Work area boundary
    ELEMENT_OBSTACLE,      // Obstacle
    ELEMENT_CHARGING_PATH, // Charging path
    ELEMENT_TRANSFER_PATH, // Transfer path
} ELEMENT_TYPE_E;

/*Map element status*/
typedef enum {
    ELEMENT_STATUS_UNKNOWN = 0, // Unknown status
    ELEMENT_STATUS_STABLE,      // Stable
    ELEMENT_STATUS_MAPPING,     // Mapping
} ELEMENT_STATUS_E;

/*Path type*/
typedef enum {
    PATH_TYPE_UNKNOWN = 0, // Unknown mode
    PATH_TYPE_STRAIGHT,    // Straight path
    PATH_TYPE_CURVED,      // Curved path
    PATH_TYPE_SPIRAL,      // Spiral path
} PATH_TYPE_E;

/*Request type enumeration*/
typedef enum {
    TYPE_GET = 0, // Get request
    TYPE_SET = 1, // Set request
} REQUEST_TYPE_E;

/*Robot configuration request work mode*/
typedef enum {
    CONFIG_UNKNOWN = 0, // Unknown type, when GET, pass this type, device needs to return all configurations
    BASE_STATION,       // Base station setting    
    RAINY_DAY_MODE,     // Rainy day mode setting
    MANUAL_SPEED,       // Manual speed setting
    CUTTING_SWITCH,     // Cutting switch setting
    PASSWORD,           // Password setting
} ROBOT_CONFIG_MODE_E;

/*Manual speed setting*/
typedef enum {
    SPEED_GENTLE = 0, // Slow speed
    SPEED_NORMAL,     // Standard
    SPEED_QUICK,      // Fast
} MANUAL_SPEED_E;

/*Task operation mode setting*/
typedef enum {
    MODE_UNKNOWN = 0,   // Unknown mode
    MODE_ECONOMY,       // Economy mode
    MODE_STANDARD,      // Standard mode
    MODE_INTENSIVE,     // Intensive mode
} TASK_SET_MODE_E;

/*Name*/
typedef struct {
    int len;    // string length
    char* name; // string memory points
} RLM_STR_ELEMENT_S;

// Fault type (device fixed fault code definition)
typedef enum  {
    FAULT_NONE = 0,                                  // No fault
    FAULT_LEFT_MOTOR_STALL = 1,                      // Left motor stall
    FAULT_RIGHT_MOTOR_STALL = 2,                     // Right motor stall
    FAULT_CUTTER_MOTOR_STALL = 3,                    // Cutter motor stall
    FAULT_LEFT_MOTOR_HALL_FAULT = 4,                 // Left motor hall fault
    FAULT_RIGHT_MOTOR_HALL_FAULT = 5,                // Right motor hall fault
    FAULT_CUTTER_MOTOR_HALL_ERROR = 6,               // Cutter motor hall error
    FAULT_MOTOR_COMM_FAILURE = 7,                    // Motor driver communication failure
    FAULT_BATTERY_DISCHARGE_FAILURE = 8,             // Battery discharge communication failure
    FAULT_BATTERY_LOW_POWER = 9,                     // Battery voltage low
    FAULT_BATTERY_DISCHARGE_OVER_CURRENT = 10,       // Battery discharge over current
    FAULT_BATTERY_DAMAGED = 11,                      // Battery error
    FAULT_BATTERY_NEED_RESET = 12,                   // Battery abnormal, needs restart
    FAULT_IMU_FAILURE = 13,                          // IMU error
    FAULT_TILTING = 14,                              // Tilting
    FAULT_OUTSIDE = 15,                              // Outside boundary
    FAULT_LIFT = 16,                                 // Lifted
    FAULT_TRAPPED = 17,                              // Trapped
    FAULT_OVERTURNED = 18,                           // Overturned
    FAULT_AI_INVALID = 19,                           // AI error
    FAULT_CROSS_FAILED = 20,                         // Zone crossing failed
    FAULT_MAPPING_FAILED = 21,                       // Mapping failed
    FAULT_SOC_DISCONNECT = 22,                       // SOC communication error
    FAULT_RETURN_CHARGE_POLE_CONNECT = 23,           // Charging connection failed
} ROBOT_FAULT_TYPE_E;
/*Error parameter reply*/
typedef struct {
    // Error code classification defined by developer as follows:
    // 0-999: Success and general errors
    // 1000-1999: Parameter validation errors
    // 2000-2999: Business logic errors
    // 3000-3999: Device status errors
    // 4000-4999: Network communication errors
    // 5000-5999: System internal errors
    int code;                   // Error code
    SEVERITY_LEVEL_E level;     // Error status level
    RLM_STR_ELEMENT_S message;  // Error reason
    RLM_STR_ELEMENT_S solutions;// Solution
    bool success;               // Success flag
} DEV_ERROR_STATUS;

/*Robot subscribe status type*/
typedef enum {
    SUBSCRIBE_UNKNOWN = 0,  // Unknown
    SUBSCRIBE_LOCALIZATION, // Localization status
    SUBSCRIBE_BATTERY,      // Battery status
    SUBSCRIBE_TASK,         // Task status
    SUBSCRIBE_PLANNED_PATH, // Planned path status
    SUBSCRIBE_ERROR,        // Error status
    SUBSCRIBE_MAX, 
} ROBOT_SUBSCRIBE_STATUS_E;

/*Mower immediate task control structure*/
typedef struct {
  float linear_x;  // Linear velocity ratio coefficient, range [-1, 1], actual linear velocity is this value multiplied by configured remote walking speed
  float angular_z; // Angular velocity, range [-1, 1], unit rad/s
} RLM_REMOTE_CTRL_TASK_S;

/*Current mowing mode*/
typedef enum {
    MODE_GLOBAL_MOWING = 0, // Global mowing
    MODE_AREA_MOWING,       // Area mowing
    MODE_EDGE_MOWING,       // Edge mowing
    MODE_REMOTE_MOWING,     // Remote mowing
    MODE_ALIGN_EDGE_MOWING,   // Align-edge cutting (ride-edge)
    MODE_CUSTOM_SHAPE_MOWING, // Custom shape pattern mowing
} OPERATION_MOW_MODE_E;

/*Mower mowing task structure*/
typedef struct {
    bool ctrl_switch_go;           // Start/stop
    OPERATION_MOW_MODE_E mow_mode; // Mowing mode
    int area_num;                  // Number of areas
    int *area_id;                  // Area id
} RLM_IMMEDIATE_MOWING_TASK_S;

/*Mower mowing task reply structure*/
typedef struct {
    bool ctrl_switch_go;           // Start/stop
    OPERATION_MOW_MODE_E mow_mode; // Mowing mode
    int area_num;                  // Number of areas
    int *area_id;                  // Area id
    DEV_ERROR_STATUS code_error;   // Error information
} RLM_IMMEDIATE_MOWING_TASK_R;

/*Mower charging task structure*/
typedef struct {
    bool ctrl_switch_charge;       // Start/stop charging
} RLM_IMMEDIATE_CHARGE_TASK_S;

/*Mower charging task reply structure*/
typedef struct {
    bool ctrl_switch_charge;       // Start/stop charging
    DEV_ERROR_STATUS code_error;   // Error information
} RLM_IMMEDIATE_CHARGE_TASK_R;

/*Mower pause task structure*/
typedef struct {
    bool ctrl_pause;       // Pause/resume
} RLM_IMMEDIATE_PAUSE_TASK_S;

/*Mower pause task reply structure*/
typedef struct {
    bool ctrl_pause;       // Pause/resume
    DEV_ERROR_STATUS code_error; // Error information
} RLM_IMMEDIATE_PAUSE_TASK_R;

/*Mower status subscription structure*/
typedef struct {
    int num;             // Number of subscriptions
    int* Subscribe_type; // Subscribe type from ROBOT_SUBSCRIBE_STATUS_E enumeration
    int timeout_sec;     // Subscription timeout
} RLM_SUBSCRIBE_STATUS_S;

/*Mower status subscription reply structure*/
typedef struct {
    bool status;                 // Reply status true/false
    DEV_ERROR_STATUS code_error; // Error information
} RLM_SUBSCRIBE_STATUS_R;

/*Geographic coordinate point*/
typedef struct {
    float latitude;  // Latitude, unit: radians
    float longitude; // Longitude, unit: radians
    float height;    // Height, unit: meters
} RLM_GEODETICPOINT_S;

/*2D point coordinates*/
typedef struct {
    float x; // x-axis coordinate
    float y; // y-axis coordinate
} RLM_POINT2D_S;

/*Pose: 2D position and yaw heading*/
typedef struct {
    RLM_POINT2D_S position; // x,y position in meters
    int heading;            // yaw in 0.1 degree units, range 0-3599 (0°~359.9°)
} RLM_POSE_S;

/*Localization status push structure*/
typedef struct {
    RLM_GEODETICPOINT_S position;     // Real-time robot geographic position 
    RLM_POSE_S robot_pose;            // Mower latest pose
    RLM_POSE_S charge_station;        // Charging station pose
    bool ready;                       // Whether localization is ready
    bool satellite_signal_detecting;  // Whether detecting satellite signal
} RLM_LOCALIZATION_R;

/*Battery status data*/
typedef struct {
    float voltage;      // Voltage (volts)
    float current;      // Current (amperes)
    int battery_level;  // Remaining battery percentage
    bool is_charging;   // Whether in charging state
} RLM_BATTERY_DATA_R;

/*Task status push*/
typedef struct {
    // Task basic information
    TASK_STATUS_E state;   // Task status
    int task_id;           // Task unique ID
    // Current active area list
    int n_active_areas;
    int *active_areas;
    int n_worked_boundary_uuid; // Number of completed work area IDs
    int *worked_boundary_uuid;  // Completed work area ID list
    // Work sequence
    int n_work_sequence;
    int *work_sequence;
    OPERATION_MOW_MODE_E mow_mode;    // Mowing mode
    bool cutting_switch;        // Cutting switch status
    int progress;   // Mowing progress
    int maparea;    // Mowing area
    int total_distance; // Total mowing distance unit: km
    int total_time; // Total mowing time unit: h
} RLM_TASK_DATA_R;

/*Planned path status push*/
typedef struct {
    int n_path_points;
    RLM_POINT2D_S *path_points;
    int current_point_index;  // Current point index
} RLM_PLANNED_PATH_DATA_R;

/*Error status push*/
typedef struct {
    int num; // Number of error codes
    DEV_ERROR_STATUS* code_error; // Error information
} RLM_ERROR_DATA_R;

/*Map editing status*/
typedef enum {
    START_EDITING = 0, // Start editing
    EXIT_EDITING,      // Exit editing
    CANCEL_EDITING,    // Cancel editing
} MAP_EDIT_STATUS_E;

/*Map recording work status*/
typedef enum {
    START_RECORDING_STATUS = 0, // Start
    STOP_RECORDING_STATUS,      // Stop
} MAP_RECORDING_WORK_STATUS_E;

/*Vector map recording mode*/
typedef enum {
    MANUAL_SAMPLING = 0,    // Manual recording point mode
    AUTO_SAMPLING,          // Auto sampling mode
    VISUAL_SAMPLING,        // Visual sampling mode
} RECORD_MODE_E;

/*Map undo operation mode*/
typedef enum {
    START_START_AUTO_UNDO = 0, // Start auto undo
    STOP_START_AUTO_UNDO,      // Stop auto undo
    LAST_POINT_UNDO,           // Undo last recorded point
} MAP_UNDO_MODE_E;

/*Map sampling operation mode*/
typedef enum {
    START_AUTO_SAMPLING = 0, // Start auto sampling
    STOP_AUTO_SAMPLING,      // Stop auto sampling
    MANUAL_SAMPLE_POINT,     // Manual trigger sampling one point
} MAP_SAMPLE_MODE_E;

/*Map operation undo record point structure*/
typedef struct {
    MAP_UNDO_MODE_E undo_mode;  // Undo mode
} MAP_UNDO_POINT_S;

/*Auto collection configuration parameter structure*/
typedef struct {
    MAP_SAMPLE_MODE_E mode;    // Sampling mode
} RLM_SAMPLING_CONFIG_S;  

/*Recording configuration parameter structure*/
typedef struct {
    MAP_RECORDING_WORK_STATUS_E status; // Work status
    RECORD_MODE_E mode;                 // Recording mode
    ELEMENT_TYPE_E element_type;        // Map element type
} RLM_RECORDING_CONFIG_S;  

/*Recording configuration parameter reply structure*/
typedef struct {
    MAP_RECORDING_WORK_STATUS_E status; // Work status
    RECORD_MODE_E mode;                 // Recording mode
    ELEMENT_TYPE_E element_type;        // Map element type
    DEV_ERROR_STATUS code_error;        // Error information
} RLM_RECORDING_CONFIG_R;  

/*Editing session parameter structure*/
typedef struct {
    int map_id;                      // Map ID when editing
    MAP_EDIT_STATUS_E edit_status;   // Editing status
} RLM_EDIT_SESSION_S;  

/*Delete element parameter structure*/
typedef struct {
  int n_element_ids;
  int *element_ids;      // Element ID list to delete
  ELEMENT_TYPE_E element_type; // Element type
} DELETE_ELEMENT_S;  

/*Delete element parameter reply structure*/
typedef struct {
  int n_element_ids;
  int *element_ids;      // Element ID list to delete
  ELEMENT_TYPE_E element_type; // Element type
  DEV_ERROR_STATUS code_error; // Error information
} DELETE_ELEMENT_R;  

/*Vector map sampling point reply structure*/
typedef struct {
    MAP_SAMPLE_MODE_E mode;     // Sampling mode
    RLM_POINT2D_S point;        // Sampled point coordinates
    DEV_ERROR_STATUS code_error; // Error information
} SAMPLED_POINT_R;

/*Vector map editing status reply structure*/
typedef struct {
    MAP_EDIT_STATUS_E edit_status;   // Editing status
    bool ready_to_sample;            // Ready to sample
    bool auto_sampling_active;       // Auto sampling active
    bool is_auto_sampling_complete;  // Auto sampling complete
    int point_count;                 // Current recorded point count
    ELEMENT_TYPE_E current_element_type; // Current recorded element type
    bool is_editing;                     // Currently editing
    bool is_auto_undo_active;            // Auto undo active
    DEV_ERROR_STATUS code_error;         // Error information
} RLM_EDIT_STATUS_R;

/*Undo information reply structure*/
typedef struct {
  MAP_UNDO_MODE_E undo_mode;   // Undo mode
  int remaining_points;
  DEV_ERROR_STATUS code_error; // Error information
} RLM_REMAINING_POINTS_R;      // Remaining points after undo

/*Map element property structure*/
typedef struct {
  RLM_STR_ELEMENT_S key;
  RLM_STR_ELEMENT_S value;
} PROPERTIES_ENTRY;  

/*Map element structure*/
typedef struct {
    int element_id;               // Element ID
    ELEMENT_TYPE_E element_type;  // Map element type
    int n_points;
    RLM_POINT2D_S *points;        // Map element point coordinate set
    int n_properties;
    PROPERTIES_ENTRY *properties; // Property set
    ELEMENT_STATUS_E status;      // Map element status
} MAP_ELEMENT_S;

/*Get planned path point segment structure*/
typedef struct {
    PATH_TYPE_E path_type;    // Path type
    int n_waypoints;          // Number of path points
    RLM_POINT2D_S* waypoints; // Planned path point coordinates
} MOWER_PLANNED_PATH;

/*Get planned path point structure*/
typedef struct {
    int n_paths;                // Number of multi-segment path points
    MOWER_PLANNED_PATH* paths;  // Multi-segment planned path point data
    DEV_ERROR_STATUS code_error; // Error information
} RLM_PLANNED_PATH_R;

/* Path point parameter structure */
typedef struct {
    RLM_POINT2D_S position; // Position coordinates
} PATH_POINT_S;

/* Path point set structure */
typedef struct {
    int n_points;          // Number of path points
    PATH_POINT_S* points;  // Path point parameters
} RLM_MOWING_PATH_S;

/* Area mowing path structure */
typedef struct {
    int area_id;                   // Area ID
    int n_paths;                   // Number of paths
    RLM_MOWING_PATH_S* paths;      // Path parameters
} RLM_AREA_MOWING_PATH_S;

/* Mowing path reply structure */
typedef struct {
    int n_area_paths;                  // Number of area paths
    RLM_AREA_MOWING_PATH_S* area_paths;// Area path data
    DEV_ERROR_STATUS code_error;       // Error code
} RLM_MOWING_PATHS_R;

/*
 * Schedule WorkPeriod mowing mode
 * Values 0..3;invalid wire values decode to STANDARD in SDK.
 */
typedef enum {
    RLM_SCHEDULE_MOW_MODE_STANDARD = 0,
    RLM_SCHEDULE_MOW_MODE_EDGE = 1,
    RLM_SCHEDULE_MOW_MODE_ALIGN_EDGE = 2,
    RLM_SCHEDULE_MOW_MODE_CUSTOM_SHAPE = 3,
} RLM_SCHEDULE_MOWING_MODE_E;


/*Schedule time period structure*/
typedef struct {
    int start_minute;     // Schedule start time (0-1440) minutes
    int end_minute;       // Schedule end time (max 1440) minutes
    bool enabled;         // Schedule enable
    int period_id;        // Auto-generated ID by device 
    bool edge_mowing;     // Whether edge mowing
    int area_ids_count;   // Area ID count
    int* area_ids;        // Area ID list
    /*
     * v1.1 optional fields (empty / unset semantics):
     * - name: unset when name[0] == '\0'; keep buffer NUL-terminated when partially filled.
     * - plan_id: unset / none when 0 .
     * - mowing_mode: optional; invalid values from wire map to STANDARD.
     */
    char name[RLM_SCHEDULE_PLAN_NAME_MAX]; /* UTF-8 plan name  */
    int plan_id;                            /* Plan group id; 0 = none */
    RLM_SCHEDULE_MOWING_MODE_E mowing_mode; /* Schedule mowing mode */
} SCHEDULE_TIMER_PERIOD;

/*Single day schedule list structure*/
typedef struct {
    DAYOFWEEK_E week;                   // Day of week
    int time_num;                       // Number of time periods
    SCHEDULE_TIMER_PERIOD* time_period; // Schedule time periods
} SCHEDULE_ALONE_PARAM;

/*Calendar timer list reply structure*/
typedef struct {
    int day_num;                 // Number of days
    SCHEDULE_ALONE_PARAM* timer; // All timer parameters
    DEV_ERROR_STATUS code_error; // Error information
} RLM_SCHEDULE_LIST_TIMER_R;

/*Create schedule parameter structure*/
typedef struct {    
    DAYOFWEEK_E week;     // Day of week
    int start_minute;     // Schedule start time (0-1440) minutes
    int end_minute;       // Schedule end time (max 1440) minutes
    bool enabled;         // Schedule enable
    bool edge_mowing;     // Whether edge mowing
    int area_ids_count;   // Area ID count
    int* area_ids;        // Area ID list
    /* v1.1 optional: name empty when name[0]=='\0'; plan_id 0 = none; mowing_mode 0..3 */
    char name[RLM_SCHEDULE_PLAN_NAME_MAX]; /* v1.1 */
    int plan_id;                            /* v1.1 */
    RLM_SCHEDULE_MOWING_MODE_E mowing_mode; /* v1.1 */
} RLM_SCHEDULE_CREATE_PARAM_S;

/*Update schedule parameter structure*/
typedef struct {  
    DAYOFWEEK_E week;                  // Day of week
    SCHEDULE_TIMER_PERIOD time_period; // Schedule time period
} RLM_SCHEDULE_UPDATE_PARAM_S;

/*Delete schedule parameter structure*/
typedef struct {
    int period_id;    // Delete directly by ID
    DAYOFWEEK_E week; // Day of week
} RLM_SCHEDULE_DELETE_PARAM_S;

/*Timer switch parameter structure*/
typedef struct {
    int period_id;     // Auto-generated ID by device 
    DAYOFWEEK_E week;  // Day of week
    bool switch_state; // Switch state
} RLM_SCHEDULE_SWITCH_PARAM_S;

/*Timer status reply structure*/
typedef struct {
    CALENDAR_OPERATION_E op; // Operation mode
    DAYOFWEEK_E week;        // Day of week       
    int period_id;           // Auto-generated ID by device      
    int start_minute;        // Schedule start time (0-1440) minutes
    int end_minute;          // Schedule end time (max 1440) minutes
    bool enabled;            // Schedule enable   
    bool edge_mowing;     // Whether edge mowing
    int area_ids_count;   // Area ID count
    int* area_ids;        // Area ID list
    
    char name[RLM_SCHEDULE_PLAN_NAME_MAX];
    int plan_id;
    RLM_SCHEDULE_MOWING_MODE_E mowing_mode;
    DEV_ERROR_STATUS code_error; // Error information
} RLM_SCHEDULE_STATUS_PARAM_R;

/*Mower base station number setting structure*/
typedef struct {
    RLM_STR_ELEMENT_S base_station_id; // Base station number
} RLM_BASE_STATION_CONFIG_S;

/*Mower rainy day mode switch setting structure*/
typedef struct {
    bool rainy_day_enabled;            // Rainy day mode switch
} RLM_RAINY_DAY_CONFIG_S;

/*Mower cutting switch setting structure*/
typedef struct {
    bool cutting_switch;            // Cutting switch setting
} RLM_CUTTING_SWITCH_CONFIG_S;

/*Mower manual speed setting structure*/
typedef struct {
    MANUAL_SPEED_E speed;              // Manual speed setting
} RLM_MANUAL_SPEED_CONFIG_S;

/*Mower password setting structure*/
typedef struct {
    bool password_enabled;              // Password setting enable
    RLM_STR_ELEMENT_S password;         // Password
    RLM_STR_ELEMENT_S password_hint;    // Password hint
} RLM_PASSWORD_CONFIG_S;

/*Mower configuration reply structure*/
typedef struct {
    RLM_STR_ELEMENT_S base_station_id; // Base station number
    bool rainy_day_enabled;            // Rainy day mode switch
    bool cutting_switch;               // Cutting switch setting
    MANUAL_SPEED_E speed;              // Manual speed setting
    bool is_password_set;              // Whether password is set
    bool password_enabled;              // Password setting enable
    RLM_STR_ELEMENT_S password;         // Password
    RLM_STR_ELEMENT_S password_hint;    // Password hint
    DEV_ERROR_STATUS code_error;       // Error information
} RLM_CONFIGURATION_R;

/*Mower base station number reply structure*/
typedef struct {
    RLM_STR_ELEMENT_S base_station_id; // Base station number
    DEV_ERROR_STATUS code_error;       // Error information
} RLM_BASE_STATION_CONFIG_R;

/*Mower rainy day mode switch reply structure*/
typedef struct {
    bool rainy_day_enabled;            // Rainy day mode switch
    DEV_ERROR_STATUS code_error;       // Error information
} RLM_RAINY_DAY_CONFIG_R;

/*Mower cutting switch reply structure*/
typedef struct {
    bool cutting_switch;            // Cutting switch
    DEV_ERROR_STATUS code_error;       // Error information
} RLM_CUTTING_SWITCH_CONFIG_R;

/*Mower manual speed reply structure*/
typedef struct {
    MANUAL_SPEED_E speed;              // Manual speed
    DEV_ERROR_STATUS code_error;       // Error information
} RLM_MANUAL_SPEED_CONFIG_R;

/*Mower password reply structure*/
typedef struct {
    bool is_password_set;               // Whether password is set
    bool password_enabled;              // Password setting enable
    RLM_STR_ELEMENT_S password;         // Password
    RLM_STR_ELEMENT_S password_hint;    // Password hint
    DEV_ERROR_STATUS code_error;        // Error information
} RLM_PASSWORD_CONFIG_R;

/*
 * Path pattern
 */
typedef enum {
  PATH_PATTERN_UNKNOWN = 0, // Unknown mode
  PATH_PATTERN_PARALLEL,    // Parallel mode
  PATH_PATTERN_SPIRAL,      // Spiral mode
  PATH_PATTERN_RANDOM       // Random mode
} PATH_PATTERN_E;

/*
 * Direction switch period
 */
typedef enum  {
  PERIOD_WEEKLY,         // Switch weekly
  PERIOD_DAILY,          // Switch daily
  PERIOD_MONTHLY         // Switch monthly
} DIRECTION_SWITCH_PERIOD_E;

/*
 * Mowing direction switch configuration
 */
typedef struct 
{
  bool enabled;  // Whether to enable direction switch
  DIRECTION_SWITCH_PERIOD_E period;  // Switch period
  int interval_value;   // Interval value (how many weeks/days/months)
}MOW_DIRECTIONSWITCH;

/*Task parameter structure*/
typedef struct {
    int area_id; // Area ID
    TASK_SET_MODE_E mode; // Task operation mode
    PATH_PATTERN_E path_mode; // Path pattern
    int cutting_height; // Working height (mm)
    bool edge_cutting;  // Whether edge cutting
    int direction; // Working direction(0-360°)
    int speed; // Speed (m/s)
    int overlap_percentage; // Overlap percentage (0-100)
    MOW_DIRECTIONSWITCH direction_switch;  // Mowing direction switch configuration
} TASK_SETTING_PARAM;

/*Task setting structure*/
typedef struct {
    int task_num;              // Number of tasks      
    TASK_SETTING_PARAM *param; // Task parameters
} RLM_TASK_SETTING_S;

/*Set task reply structure*/
typedef struct {
    REQUEST_TYPE_E request_type;  // Request type
    int task_num;              // Number of tasks      
    TASK_SETTING_PARAM *param; // Task parameters
    DEV_ERROR_STATUS code_error; // Error information
} RLM_TASK_SETTING_R;

typedef struct RLM_CUSTOM_SHAPE_S RLM_CUSTOM_SHAPE_S;

/*Get map information structure*/
typedef struct {
    int map_id;              // Get corresponding map information by map id      
} RLM_MAP_INFO_S;

/*Get map information reply structure*/
typedef struct {
    int map_id;              // Get corresponding map information by map id      
    RLM_STR_ELEMENT_S map_name;  // Map name
    int element_num; // Number of map elements
    MAP_ELEMENT_S* elements; // All map element parameters
    DEV_ERROR_STATUS code_error; // Error information
    int n_custom_shapes; /**< Custom shapes for this map; encoded as RLM_CUSTOM_SHAPE_S.custom_shapes (field 4). */
    RLM_CUSTOM_SHAPE_S *custom_shapes; /**< NULL if n_custom_shapes==0; caller-owned until ty_rlm_map_info_response returns. */
} RLM_MAP_INFO_R;

/*Delete map information structure*/
typedef struct {
    int map_id;              // Delete corresponding map information by map id      
} RLM_MAP_DELETE_S;

/*Delete map information reply structure*/
typedef struct {
  int map_id;              // Delete corresponding map information by map id   
  DEV_ERROR_STATUS code_error; // Error information
} RLM_MAP_DELETE_R;

/*Get mowing record information structure*/
typedef struct {
    int record_id;              // Get complete mowing record information by record id      
} RLM_RECORD_INFO_S;

/*Get mowing record information reply structure*/
typedef struct {
    int record_id;              // Get complete mowing record information by record id
    RLM_MAP_INFO_R map_info;    // Map information
    int n_paths; // Number of path points
    MOWER_PLANNED_PATH* paths; // Planned path point data 
    DEV_ERROR_STATUS code_error; // Error information
} RLM_RECORD_INFO_R;

/*
 * Query record
 */
typedef struct
{
  int start_time;    // Start time
  int end_time;      // End time
  int record_num;     // Number of records
}QUERY_RECORD_S;

/*
 * Mowing record summary
 */
typedef struct
{
  int record_id;     // Record ID
  int start_time;    // Start time
  int end_time;      // End time
  OPERATION_MOW_MODE_E mow_mode;    // Mowing mode
  MOWING_RESULT_E stop_reason;   // Stop reason
  int n_area_names;    // Number of area names
  RLM_STR_ELEMENT_S *area_names;     // Cleaning area name list
  int mow_area;    // Cleaning area unit: square meters
}MOW_RECORD_SUMMARY;

/*Get mowing record list information structure*/
typedef struct {
    int n_mow_records;
    MOW_RECORD_SUMMARY *mow_records;  // Mowing record list
    DEV_ERROR_STATUS code_error; // Error information
} RLM_MOW_RECORD_LIST_R;

/*
 * Error record summary
 */
typedef struct
{
  int record_id;     // Record ID
  int error_time;    // Time when error occurred
  RLM_STR_ELEMENT_S error_reason;  // Error reason
  bool is_resolved;    // Whether error is resolved flag
  int error_code;// Error code
  RLM_STR_ELEMENT_S error_description;// Error description
}ERROR_LOG_RECORD_SUMMARY;

/*Get error record list information structure*/
typedef struct {
    int n_error_log_records;
    ERROR_LOG_RECORD_SUMMARY *error_log_records;  // Error record list
    DEV_ERROR_STATUS code_error; // Error information
} RLM_ERROR_LOG_RECORD_LIST_R;

/*Map summary information structure*/
typedef struct {
  int map_id;  // Map ID
  int last_modified;   // Last modified time, UTC timestamp (seconds)
  int element_count;   // Element count
  bool is_editing;     // Whether currently editing
  RLM_STR_ELEMENT_S version;   // Map version
} MAP_SUMMARY;

/*Get map list information reply structure*/
typedef struct {
    int n_maps;
    MAP_SUMMARY *maps; // Available map list
    DEV_ERROR_STATUS code_error; // Error information
} RLM_MAP_LIST_INFO_R;

/*Custom shape type*/
typedef enum {
    CUSTOM_SHAPE_UNKNOWN = 0,
    CUSTOM_SHAPE_HEART,       // Heart shape
    CUSTOM_SHAPE_CIRCLE,      // Circle shape
    CUSTOM_SHAPE_TRIANGLE,    // Triangle shape
    CUSTOM_SHAPE_STAR,        // Star shape
    CUSTOM_SHAPE_RECTANGLE,   // Rectangle shape
    CUSTOM_SHAPE_OTHER,       // Custom type, requires alignment with APP
} CUSTOM_SHAPE_TYPE_E;

/*Custom shape transform*/
typedef struct {
    RLM_POINT2D_S center;  // Shape center point in map coordinates
    int width;             // Width (mm)
    int height;            // Height (mm)
    int rotation;          // Rotation angle (0.1 degree unit, range 0-3599)
} RLM_SHAPE_TRANSFORM_S;

/*Custom shape structure*/
struct RLM_CUSTOM_SHAPE_S {
    int shape_id;                    // Shape unique ID
    CUSTOM_SHAPE_TYPE_E shape_type;  // Shape type
    RLM_SHAPE_TRANSFORM_S transform; // Transform info (position, size, rotation)
    int n_contour_points;            /**< Number of map-space contour vertices (APP-computed polygon). */
    RLM_POINT2D_S *contour_points;   /**< Map coordinates (e.g. mm); heap owned by caller/store when echoing. */
    int map_id;                      /**< Map ID this shape belongs to. */
    int n_local_points;              /**< Count of normalized local editor points (-5000~5000 scale). */
    RLM_POINT2D_S *local_points;     /**< Local shape template without placement; heap owned by caller/store. */
};

/**
 * @brief SET_SHAPES application callback payload .
 * @note One callback per MQTT SET_SHAPES. `shapes` is heap-allocated by the SDK and freed after
 *       FUNC_SET_CB returns; the application must deep-copy if it needs to keep point arrays.
 */
typedef struct {
    int map_id;                   /**< Target map from SetShapesPayload.map_id (clear/replace scope). */
    int n_shapes;               /**< Count of `shapes[]`; 0 means clear all custom shapes on `map_id`. */
    RLM_CUSTOM_SHAPE_S *shapes; /**< Full list for this SET; each entry may own contour_points/local_points
                                     malloc'd by SDK until the callback returns. */
} RLM_CUSTOM_SHAPE_SET_S;

/*Custom shape response structure*/
typedef struct {
    int n_shapes;                  /**< Number of shapes in `shapes` (SET_SHAPES echo). */
    RLM_CUSTOM_SHAPE_S *shapes;    /**< Shape list; SET_SHAPES should echo final stored state */
    DEV_ERROR_STATUS code_error;   /**< Result: success flag and optional error details. */
} RLM_CUSTOM_SHAPE_R;

/***********************************************************
********************function declaration********************
***********************************************************/

/**
 * @brief lrm complex function setting callback function pointer
 * @param [COMPLEX_RLM_CMD_E] cmd, the SDK uses this parameter to notify the application of the current setting command to be processed, see the list for setting commands, all ending with SET
 * @param [void *] param, parameters corresponding to the command, the application converts the param parameter into the command corresponding parameter structure based on the cmd command field, the structure correspondence can refer to the accompanying demo
 * @note For CUSTOM_SHAPE_SET_REQUEST, `param` points to RLM_CUSTOM_SHAPE_SET_S: full list for one
 *       SET_SHAPES (see struct). Do not retain `shapes` or embedded point pointers after return.
 */
typedef OPERATE_RET (*FUNC_SET_CB)(IN COMPLEX_RLM_CMD_E cmd, IN void* param);

/**
 * @brief rlm complex function query callback function pointer
 * @param [COMPLEX_RLM_CMD_E] cmd, the SDK uses this parameter to notify the application of the current query command to be processed, see the list for query commands, all ending with QUERY
 * @param [void *] param, parameters corresponding to the command, current query command, its parameters are meaningless and do not need to be processed
 * @note  When the application gets the corresponding command parameters in the query callback function and needs to report, please call the corresponding response reporting function
 */
typedef OPERATE_RET (*FUNC_QUERY_CB)(IN COMPLEX_RLM_CMD_E cmd, IN void* param);

/**
 * @brief Register complex capability setting and query callback functions
 * @param[in] sets_handler: Command setting callback function that the application needs to implement
 * @param[in] query_handler: Command query callback function that the application needs to implement
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_complex_func_register(IN FUNC_SET_CB sets_handler, IN FUNC_QUERY_CB query_handler);

/**
 * @brief Mower Bluetooth transmission interface
 * @param[in] data: Transmission data
 * @param[in] len:  Transmission data length
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET handle_bt_rlm_data_deal(unsigned char* data, int len);

/**
 * @brief Mowing task reply interface
 * @param[in] task_res: Mowing task result reply
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_immediate_mowing_task_response(IN RLM_IMMEDIATE_MOWING_TASK_R* task_res);

/**
 * @brief Charging task reply interface
 * @param[in] task_res: Charging task result reply
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_immediate_charge_task_response(IN RLM_IMMEDIATE_CHARGE_TASK_R* task_res);

/**
 * @brief Pause task reply interface
 * @param[in] task_res: Pause task result reply
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_immediate_pause_task_response(IN RLM_IMMEDIATE_PAUSE_TASK_R* task_res);

/**
 * @brief Mower status subscription reply interface
 * @param[in] subscribe_res: Status subscription result reply
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_subscribe_response(IN RLM_SUBSCRIBE_STATUS_R* subscribe_res);

/**
 * @brief Localization information push interface
 * @param[in] localization: Localization information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_localization_push(IN RLM_LOCALIZATION_R* localization);

/**
 * @brief Battery information push interface
 * @param[in] battery: Battery information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_battery_push(IN RLM_BATTERY_DATA_R* battery);

/**
 * @brief Task information push interface
 * @param[in] task: Task information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_task_push(IN RLM_TASK_DATA_R* task);

/**
 * @brief Error information push interface
 * @param[in] error: Error information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_error_push(IN RLM_ERROR_DATA_R* error);

/**
 * @brief Mowing path information push interface
 * @param[in] planned_path: Mowing path information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_planned_path_push(IN RLM_PLANNED_PATH_DATA_R* planned_path);

/**
 * @brief Vector map editing status reply interface
 * @param[in] edit_status: Vector map editing status
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_edit_status_response(IN RLM_EDIT_STATUS_R* edit_status);

/**
 * @brief Vector map delete element reply interface
 * @param[in] deleted_elements: Vector map delete element
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_deleted_elements_response(IN DELETE_ELEMENT_R* deleted_elements);

/**
 * @brief Vector map sampling point reply interface
 * @param[in] sampled_point: Vector map sampling point information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_sampled_point_response(IN SAMPLED_POINT_R* sampled_point);

/** 
 * @brief Vector map undo information reply interface
 * @param[in] undo_info: Undo information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_undo_info_response(IN RLM_REMAINING_POINTS_R* undo_info);

/**
 * @brief Vector map record command reply interface
 * @param[in] recording_status: Status recovery
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_record_status_response(IN RLM_RECORDING_CONFIG_R *recording_status);

/**
 * @brief Planned path information reply interface
 * @param[in] planned_path: Planned path information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_planned_path_response(IN RLM_PLANNED_PATH_R* planned_path);

/**
 * @brief Mowing path information reply interface
 * @param[in] mowing_path: Mowing path information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_mowing_path_response(IN RLM_MOWING_PATHS_R* mowing_path);

/**
 * @brief Timer list information reply interface
 * @param[in] list_timer: Timer list information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_schedule_list_timer_response(IN RLM_SCHEDULE_LIST_TIMER_R* list_timer);

/**
 * @brief Timer status reply interface
 * @param[in] status_timer: Timer reply information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_schedule_status_timer_response(IN RLM_SCHEDULE_STATUS_PARAM_R* status_timer);

/**
 * @brief Mower all configuration information reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_configuration_response(IN RLM_CONFIGURATION_R* config);

/**
 * @brief Mower base station number reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_base_station_config_response(IN RLM_BASE_STATION_CONFIG_R* config);

/**
 * @brief Mower rainy day mode switch reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_rainy_day_config_response(IN RLM_RAINY_DAY_CONFIG_R* config);

/**
 * @brief Mower cutting switch reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_cutting_switch_config_response(IN RLM_CUTTING_SWITCH_CONFIG_R* config);

/**
 * @brief Mower manual speed reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_manual_speed_config_response(IN RLM_MANUAL_SPEED_CONFIG_R* config);

/**
 * @brief Mower password reply interface
 * @param[in] config: Configuration information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_password_config_response(IN RLM_PASSWORD_CONFIG_R* config);

/**
 * @brief Mower task setting information reply interface
 * @param[in] task_setting: Task setting information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_task_setting_response(IN RLM_TASK_SETTING_R* task_setting);

/**
 * @brief Mower map information report interface
 * @param[in] info_info: Map information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 * @note When `n_custom_shapes` > 0, `custom_shapes` must point to a valid array;
 */
OPERATE_RET ty_rlm_map_info_response(IN RLM_MAP_INFO_R* info_info);

/**
 * @brief Mower mowing record information report interface
 * @param[in] record_info: Mowing record information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_record_info_response(IN RLM_RECORD_INFO_R* record_info);

/**
 * @brief Mower mowing record list report interface
 * @param[in] record_list: Mowing record list information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_log_record_list_response(IN RLM_MOW_RECORD_LIST_R* record_list);

/**
 * @brief Mower error log record list report interface
 * @param[in] error_log_record_list: Error log record list information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_error_log_record_list_response(IN RLM_ERROR_LOG_RECORD_LIST_R* error_log_record_list);

/**
 * @brief Mower delete map information reply interface
 * @param[in] map_delete: Delete map information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_map_delete_response(IN RLM_MAP_DELETE_R* map_delete);

/**
 * @brief Mower map list summary information reply interface
 * @param[in] map_list: Map summary information
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_map_list_response(IN RLM_MAP_LIST_INFO_R* map_list);

/**
 * @brief Custom shape response interface
 * @param[in] shape_res: Custom shape response data
 * @return OPERATE_RET: 0 success, other error codes indicate failure
 */
OPERATE_RET ty_rlm_custom_shape_response(IN RLM_CUSTOM_SHAPE_R* shape_res);

#ifdef __cplusplus
}
#endif

#endif /* __TY_RLM_FUNC_H__ */
