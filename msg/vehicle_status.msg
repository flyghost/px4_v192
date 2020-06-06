# If you change the order, add or remove arming_state_t states make sure to update the arrays
# in state_machine_helper.cpp as well.
uint64 timestamp				# time since system start (microseconds)

# 解锁状态机
uint8 ARMING_STATE_INIT = 0
uint8 ARMING_STATE_STANDBY = 1
uint8 ARMING_STATE_ARMED = 2
uint8 ARMING_STATE_STANDBY_ERROR = 3
uint8 ARMING_STATE_REBOOT = 4
uint8 ARMING_STATE_IN_AIR_RESTORE = 5
uint8 ARMING_STATE_MAX = 6

# FailureDetector status
uint8 FAILURE_NONE = 0
uint8 FAILURE_ROLL = 1 	# (1 << 0)
uint8 FAILURE_PITCH = 2	# (1 << 1)
uint8 FAILURE_ALT = 4 	# (1 << 2)

# HIL
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# Navigation state, i.e. "what should vehicle do".
# 飞行模式
uint8 NAVIGATION_STATE_MANUAL = 0		# Manual mode                    手动飞行
uint8 NAVIGATION_STATE_ALTCTL = 1		# Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2		# Position control mode          定高
uint8 NAVIGATION_STATE_AUTO_MISSION = 3		# Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4		# Auto loiter mode               悬停
uint8 NAVIGATION_STATE_AUTO_RTL = 5		# Auto return to launch mode     返航
uint8 NAVIGATION_STATE_AUTO_RCRECOVER = 6	# RC recover mode
uint8 NAVIGATION_STATE_AUTO_RTGS = 7		# Auto return to groundstation on data link loss
uint8 NAVIGATION_STATE_AUTO_LANDENGFAIL = 8 	# Auto land on engine failure
uint8 NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9	# Auto land on gps failure (e.g. open loop loiter down)
uint8 NAVIGATION_STATE_ACRO = 10		# Acro mode                 特技飞行
uint8 NAVIGATION_STATE_UNUSED = 11		# Free slot
uint8 NAVIGATION_STATE_DESCEND = 12		# Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13		# Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15		# Stabilized mode
uint8 NAVIGATION_STATE_RATTITUDE = 16		# Rattitude (aka "flip") mode
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17	# Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18		# Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19	# Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20	# Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21       # Orbit in a circle
uint8 NAVIGATION_STATE_MAX = 22

# 遥控器输入的模式
uint8 RC_IN_MODE_DEFAULT = 0
uint8 RC_IN_MODE_OFF = 1
uint8 RC_IN_MODE_GENERATED = 2

# state machine / state of vehicle.
# Encodes the complete system state and is set by the commander app.

uint8 nav_state				# set navigation state machine to specified value  导航状态，也就是飞机飞什么模式
uint8 arming_state			# current arming state  解锁状态，所能赋值的就是上面的枚举类型
uint8 hil_state				# current hil state  是否为仿真状态
bool failsafe				# true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...) 失效保护是否为有效

uint8 system_type			# system type, contains mavlink MAV_TYPE
uint8 system_id				# system id, contains MAVLink's system ID field
uint8 component_id			# subsystem / component id, contains MAVLink's component ID field

bool is_rotary_wing			# True if system is in rotary wing configuration, so for a VTOL this is only true while flying as a multicopter

bool is_vtol				# True if the system is VTOL capable
bool vtol_fw_permanent_stab		# True if VTOL should stabilize attitude for fw in manual mode
bool in_transition_mode			# True if VTOL is doing a transition    是否为转换模式
bool in_transition_to_fw		# True if VTOL is doing a transition from MC to FW   谁转谁

bool rc_signal_lost				# true if RC reception lost
uint8 rc_input_mode				# set to 1 to disable the RC input, 2 to enable manual control to RC in mapping. 遥控器输入的模式

bool data_link_lost				# datalink to GCS lost   地面站信号丢失
uint8 data_link_lost_counter			# counts unique data link lost events

bool high_latency_data_link_lost 			# Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

bool engine_failure				# Set to true if an engine failure is detected       CPU是否坏了
bool mission_failure				# Set to true if mission could not continue/finish    任务是否结束了

uint8 failure_detector_status			# Bitmask containing FailureDetector status [0, 0, 0, 0, 0, FAILURE_ALT, FAILURE_PITCH, FAILURE_ROLL]

# see SYS_STATUS mavlink message for the following
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health

# airspeed fault and airspeed use status
float32 arspd_check_level	# integrated airspeed inconsistency as checked against the COM_TAS_FS_INTEG parameter
bool aspd_check_failing		# true when airspeed consistency checks are failing
bool aspd_fault_declared	# true when an airspeed fault has been declared
bool aspd_use_inhibit		# true if switching to a non-airspeed control mode has been requested
bool aspd_fail_rtl		# true if airspeed failure invoked RTL has been requested
float32 load_factor_ratio	# ratio of measured to aerodynamic load factor limit. Greater than 1 indicates airspeed low error condition.
