/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "../../include/turtlebot3/turtlebot3.h"

/*******************************************************************************
* Definition of dependency data according to TB3 model.
*******************************************************************************/
typedef struct TB3ModelInfo{
  const char* model_str;
  uint32_t model_info;
  float wheel_radius;
  float wheel_separation;
  float turning_radius;
  float robot_radius;
  bool has_manipulator;
} TB3ModelInfo;

static const TB3ModelInfo burger_info = {
  "Burger",
  1,
  0.033,
  0.160,
  0.080,
  0.105,
  false,
};

static const TB3ModelInfo waffle_info = {
  "Waffle",
  2,
  0.033,
  0.287,
  0.1435,
  0.220,
  false,
};

static const TB3ModelInfo waffle_with_manipulator_info = {
  "Waffle_OpenManipulator",
  3,
  0.033,
  0.287,
  0.1435,
  0.220,
  true,
};


/*******************************************************************************
* Declaration for motors
*******************************************************************************/
static Turtlebot3MotorDriver motor_driver;
static OpenManipulatorDriver manipulator_driver(motor_driver.getDxl());

static const TB3ModelInfo* p_tb3_model_info;
static float max_linear_velocity, min_linear_velocity;
static float max_angular_velocity, min_angular_velocity;
static int16_t max_pwm_value, min_pwm_value;

static float goal_velocity[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_cmd[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_rc100[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_button[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};
static int16_t goal_pwm[MortorLocation::MOTOR_NUM_MAX] = {0, 0};
static int16_t goal_pwm_from_cmd[MortorLocation::MOTOR_NUM_MAX] = {0, 0};
static int16_t goal_pwm_from_button[MortorLocation::MOTOR_NUM_MAX] = {0, 0};

static void update_goal_pwm_from_2values(void);
static void update_goal_velocity_from_3values(void);
static void test_motors_with_buttons(uint8_t buttons);
static bool get_connection_state_with_motors();
static void set_connection_state_with_motors(bool is_connected);
static bool get_connection_state_with_joints();
static void set_connection_state_with_joints(bool is_connected);

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
static Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
static Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
static Turtlebot3Controller controllers;

/*******************************************************************************
* Declaration for optional onboard-segway-controller
*******************************************************************************/
// constants
static const bool enable_onboard_segway_ctrl = false;
static const bool segway_enable_auto_trim = true;
static const bool segway_enable_integrator = true;
static const float segway_k_ext_default[4] = {-3.2030f, -6.3362f, -0.7768f, -2.7468f};
static const float segway_ts = (float)INTERVAL_MS_TO_CONTROL_MOTOR * 0.001f;
static const float segway_f_grenz = 10.0f;
static const float segway_phi_max_rad = 15.0f * (PI / 180.0f);
static const float segway_u_max = 0.22f;
static const float segway_x_err_max = 0.4f;
static const float segway_turn_gain = 60.0f;
// static const float s_dot_ref = 0.0f;
static const float segway_calib_phi_window_rad = 5.0f * (PI / 180.0f);
// static const float segway_calib_s_dot_window_mps = 0.2f;
static const uint16_t segway_calib_samples_target = 1000;
// variables
static float segway_lp_b0 = 0.0f;
static float segway_lp_b1 = 0.0f;
static float segway_lp_a1 = 0.0f;
static float segway_lp_x_prev = 0.0f;
static float segway_lp_y_prev = 0.0f;
static float segway_x_err = 0.0f;
static bool segway_active = false;
static bool segway_calib_done = false;
static bool segway_calib_wait_printed = false;
static uint16_t segway_calib_samples = 0;
static float segway_calib_sum_phi = 0.0f;
static float segway_calib_sum_s_dot = 0.0f;
static float segway_phi_trim_rad = 0.0f;
static float segway_s_dot_bias_mps = 0.0f;

static void init_segway_controller(void);
static void update_goal_pwm_from_segway_controller(void);

/*******************************************************************************
* Declaration for DYNAMIXEL Slave Function
*******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

static void dxl_slave_write_callback_func(uint16_t addr, uint8_t &dxl_err_code, void* arg);

static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node();

static void update_imu(uint32_t interval_ms);
static void update_times(uint32_t interval_ms);
static void update_gpios(uint32_t interval_ms);
static void update_motor_status(uint32_t interval_ms);
static void update_battery_status(uint32_t interval_ms);
static void update_analog_sensors(uint32_t interval_ms);
static void update_joint_status(uint32_t interval_ms);


DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE);

enum ControlTableItemAddr{
  ADDR_MODEL_INFORM    = 2,
  
  ADDR_MILLIS          = 10,

  ADDR_DEBUG_MODE      = 14,  
  ADDR_CONNECT_ROS2    = 15,
  ADDR_CONNECT_MANIP   = 16,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,

  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,
  
  ADDR_PRESENT_CURRENT_L  = 120,
  ADDR_PRESENT_CURRENT_R  = 124,
  ADDR_PRESENT_VELOCITY_L = 128,
  ADDR_PRESENT_VELOCITY_R = 132,
  ADDR_PRESENT_POSITION_L = 136,
  ADDR_PRESENT_POSITION_R = 140,
  
  ADDR_MOTOR_CONNECT      = 148,
  ADDR_MOTOR_TORQUE       = 149,
  ADDR_CMD_VEL_LINEAR_X   = 150,
  ADDR_CMD_VEL_LINEAR_Y   = 154,
  ADDR_CMD_VEL_LINEAR_Z   = 158,
  ADDR_CMD_VEL_ANGULAR_X  = 162,
  ADDR_CMD_VEL_ANGULAR_Y  = 166,
  ADDR_CMD_VEL_ANGULAR_Z  = 170,
  ADDR_PROFILE_ACC_L      = 174,
  ADDR_PROFILE_ACC_R      = 178,

  ADDR_OPERATING_MODE     = 182,
  ADDR_PRESENT_PWM_L      = 183, // not implented yet
  ADDR_PRESENT_PWM_R      = 185, // not implented yet
  ADDR_CMD_PWM_L          = 187,
  ADDR_CMD_PWM_R          = 189,

  ADDR_TORQUE_JOINT             = 199,

  ADDR_GOAL_POSITION_JOINT_1    = 200,
  ADDR_GOAL_POSITION_JOINT_2    = 204,
  ADDR_GOAL_POSITION_JOINT_3    = 208,
  ADDR_GOAL_POSITION_JOINT_4    = 212,
  ADDR_GOAL_POSITION_GRIPPER    = 216,
  ADDR_GOAL_POSITION_WR_JOINT   = 220,
  ADDR_GOAL_POSITION_WR_GRIPPER = 221,
  ADDR_GOAL_POSITION_RD         = 222,

  ADDR_PRESENT_POSITION_JOINT_1 = 224,
  ADDR_PRESENT_POSITION_JOINT_2 = 228,
  ADDR_PRESENT_POSITION_JOINT_3 = 232,
  ADDR_PRESENT_POSITION_JOINT_4 = 236,
  ADDR_PRESENT_POSITION_GRIPPER = 240,

  ADDR_PRESENT_VELOCITY_JOINT_1 = 244,
  ADDR_PRESENT_VELOCITY_JOINT_2 = 248,
  ADDR_PRESENT_VELOCITY_JOINT_3 = 252,
  ADDR_PRESENT_VELOCITY_JOINT_4 = 256,
  ADDR_PRESENT_VELOCITY_GRIPPER = 260,

  ADDR_PRESENT_CURRENT_JOINT_1  = 264,
  ADDR_PRESENT_CURRENT_JOINT_2  = 266,
  ADDR_PRESENT_CURRENT_JOINT_3  = 268,
  ADDR_PRESENT_CURRENT_JOINT_4  = 270,
  ADDR_PRESENT_CURRENT_GRIPPER  = 272,

  ADDR_PROFILE_ACC_JOINT_1      = 284,
  ADDR_PROFILE_ACC_JOINT_2      = 288,
  ADDR_PROFILE_ACC_JOINT_3      = 292,
  ADDR_PROFILE_ACC_JOINT_4      = 296,
  ADDR_PROFILE_ACC_GRIPPER      = 300,
  ADDR_PROFILE_ACC_WR_JOINT     = 304,
  ADDR_PROFILE_ACC_WR_GRIPPER   = 305,
  ADDR_PROFILE_ACC_RD           = 306,

  ADDR_PROFILE_VEL_JOINT_1      = 308,
  ADDR_PROFILE_VEL_JOINT_2      = 312,
  ADDR_PROFILE_VEL_JOINT_3      = 316,
  ADDR_PROFILE_VEL_JOINT_4      = 320,
  ADDR_PROFILE_VEL_GRIPPER      = 324,
  ADDR_PROFILE_VEL_WR_JOINT     = 328,
  ADDR_PROFILE_VEL_WR_GRIPPER   = 329,
  ADDR_PROFILE_VEL_RD           = 330,

  ADDR_GOAL_CURRENT_JOINT_1     = 332,
  ADDR_GOAL_CURRENT_JOINT_2     = 334,
  ADDR_GOAL_CURRENT_JOINT_3     = 336,
  ADDR_GOAL_CURRENT_JOINT_4     = 338,
  ADDR_GOAL_CURRENT_GRIPPER     = 340,  
  ADDR_GOAL_CURRENT_WR_JOINT    = 342,
  ADDR_GOAL_CURRENT_WR_GRIPPER  = 343,
  ADDR_GOAL_CURRENT_RD          = 344,

  ADDR_SEGWAY_CTRL_ENABLE = 348,
  ADDR_SEGWAY_K_EXT_1     = 352,
  ADDR_SEGWAY_K_EXT_2     = 356,
  ADDR_SEGWAY_K_EXT_3     = 360,
  ADDR_SEGWAY_K_EXT_4     = 364,

};

typedef struct ControlItemVariables{
  uint32_t model_inform;

  uint32_t dev_time_millis;
  uint32_t dev_time_micros;

  int8_t device_status;
  uint8_t heart_beat;
  bool debug_mode;
  bool is_connect_ros2_node;
  bool is_connect_motors;
  bool is_connect_manipulator;

  bool user_led[4];
  bool push_button[2];
  bool bumper[2];

  uint16_t illumination;
  uint32_t ir_sensor;
  float sornar;

  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;

  uint8_t buzzer_sound;

  bool imu_recalibration;
  float angular_vel[3];
  float linear_acc[3];
  float magnetic[3];
  float orientation[4];

  int32_t present_position[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_velocity[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_current[MortorLocation::MOTOR_NUM_MAX];

  bool motor_torque_enable_state;
  int32_t cmd_vel_linear[3];
  int32_t cmd_vel_angular[3];
  uint32_t profile_acceleration[MortorLocation::MOTOR_NUM_MAX];

  uint8_t operating_mode;
  int16_t cmd_pwm[MortorLocation::MOTOR_NUM_MAX];

  bool joint_torque_enable_state;
  joint_position_info_t joint_goal_position;  
  joint_position_info_t joint_present_position;
  joint_velocity_info_t joint_present_velocity;
  joint_current_info_t joint_present_current;
  joint_accel_info_t joint_profile_acc;
  joint_accel_info_t joint_profile_vel;
  joint_current_info_t joint_goal_current;

  bool joint_goal_position_wr_joint;
  bool joint_goal_position_wr_gripper;
  bool joint_goal_position_rd;

  bool joint_profile_acc_wr_joint;
  bool joint_profile_acc_wr_gripper;
  bool joint_profile_acc_rd;

  bool joint_profile_vel_wr_joint;
  bool joint_profile_vel_wr_gripper;
  bool joint_profile_vel_rd;

  bool joint_goal_current_wr_joint;
  bool joint_goal_current_wr_gripper;
  bool joint_goal_current_rd;

  bool segway_ctrl_enable;
  float segway_k_ext[4];

}ControlItemVariables;

static ControlItemVariables control_items;


/*******************************************************************************
* Definition for TurtleBot3Core 'begin()' function
*******************************************************************************/
void TurtleBot3Core::begin(const char* model_name)
{
  uint16_t model_motor_rpm;

  if(strcmp(model_name, "Burger") == 0 || strcmp(model_name, "burger") == 0){
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }else if(strcmp(model_name, "Waffle") == 0 || strcmp(model_name, "waffle") == 0){
    p_tb3_model_info = &waffle_info;
    model_motor_rpm = 77;
  }else if(strcmp(model_name, "Waffle_OpenManipulator") == 0){
    p_tb3_model_info = &waffle_with_manipulator_info;
    model_motor_rpm = 77;
  }else{
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }

  max_linear_velocity = p_tb3_model_info->wheel_radius*2*PI*model_motor_rpm/60;
  min_linear_velocity = -max_linear_velocity;
  max_angular_velocity = max_linear_velocity/p_tb3_model_info->turning_radius;
  min_angular_velocity = -max_angular_velocity;
  max_pwm_value = PWM_MAX;
  min_pwm_value = -PWM_MAX;

  bool ret; (void)ret;
  DEBUG_SERIAL_BEGIN(57600);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("Version : V221004R1");
  DEBUG_PRINTLN("Begin Start...");

  // Setting for Dynamixel motors
  ret = motor_driver.init();
  DEBUG_PRINTLN(ret==true?"Motor driver setup completed.":"Motor driver setup failed.");
  // Setting for IMU
  ret = sensors.init();
  DEBUG_PRINTLN(ret==true?"Sensors setup completed.":"Sensors setup failed.");
  // Init diagnosis
  ret = diagnosis.init();
  DEBUG_PRINTLN(ret==true?"Diagnosis setup completed.":"Diagnosis setup failed.");
  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  ret = controllers.init(max_linear_velocity, max_angular_velocity);
  DEBUG_PRINTLN(ret==true?"RC100 Controller setup completed.":"RC100 Controller setup failed.");

  if (p_tb3_model_info->has_manipulator == true)
  {    
    ret = manipulator_driver.init();
    DEBUG_PRINTLN(ret==true?"Manipulator driver setup completed.":"Manipulator driver setup failed.");
  }

  DEBUG_PRINT("Dynamixel2Arduino Item Max : ");
  DEBUG_PRINTLN(CONTROL_ITEM_MAX);

  if (enable_onboard_segway_ctrl == true) {
    DEBUG_PRINTLN("Onboard Segway Controller : Enabled");
  } else {
    DEBUG_PRINTLN("Onboard Segway Controller : Disabled");
  }

  control_items.debug_mode = false;
  control_items.is_connect_ros2_node = false;
  control_items.is_connect_manipulator = false;  

  control_items.operating_mode = OperatingMode::OP_VELOCITY; // OP_VELOCITY and OP_PWM available
  control_items.cmd_pwm[MortorLocation::LEFT] = 0;
  control_items.cmd_pwm[MortorLocation::RIGHT] = 0;
  control_items.segway_ctrl_enable = enable_onboard_segway_ctrl;
  memcpy(control_items.segway_k_ext, segway_k_ext_default, sizeof(control_items.segway_k_ext));

  // Port begin
  dxl_slave.begin();
  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave.setID(ID_DXL_SLAVE);

  /* Add control items for Slave */
  // Items for model information of device
  control_items.model_inform = p_tb3_model_info->model_info;
  dxl_slave.addControlItem(ADDR_MODEL_INFORM, control_items.model_inform);
  // Items for Timer of device
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);

  // Items to debug mode
  dxl_slave.addControlItem(ADDR_DEBUG_MODE, control_items.debug_mode);
  // Items to connect ros2
  dxl_slave.addControlItem(ADDR_CONNECT_ROS2, control_items.is_connect_ros2_node);
  // Items to connect manipulator
  dxl_slave.addControlItem(ADDR_CONNECT_MANIP, control_items.is_connect_manipulator);

  // Items to inform device status
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  // Items to check connection state with node
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  // Items for GPIO
  dxl_slave.addControlItem(ADDR_USER_LED_1, control_items.user_led[0]);
  dxl_slave.addControlItem(ADDR_USER_LED_2, control_items.user_led[1]);
  dxl_slave.addControlItem(ADDR_USER_LED_3, control_items.user_led[2]);
  dxl_slave.addControlItem(ADDR_USER_LED_4, control_items.user_led[3]);
  dxl_slave.addControlItem(ADDR_BUTTON_1, control_items.push_button[0]);
  dxl_slave.addControlItem(ADDR_BUTTON_2, control_items.push_button[1]);
  dxl_slave.addControlItem(ADDR_BUMPER_1, control_items.bumper[0]);
  dxl_slave.addControlItem(ADDR_BUMPER_2, control_items.bumper[1]);
  // Items for Analog sensors
  dxl_slave.addControlItem(ADDR_ILLUMINATION, control_items.illumination);
  dxl_slave.addControlItem(ADDR_IR, control_items.ir_sensor);
  dxl_slave.addControlItem(ADDR_SORNA, control_items.sornar);
  // Items for Battery
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE, control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT, control_items.bat_percent_x100);
  // Items for Buzzer
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);
  // Items for IMU
  dxl_slave.addControlItem(ADDR_IMU_RECALIBRATION, control_items.imu_recalibration);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_X, control_items.angular_vel[0]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Y, control_items.angular_vel[1]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Z, control_items.angular_vel[2]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_X, control_items.linear_acc[0]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Y, control_items.linear_acc[1]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Z, control_items.linear_acc[2]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_X, control_items.magnetic[0]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Y, control_items.magnetic[1]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Z, control_items.magnetic[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_W, control_items.orientation[0]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_X, control_items.orientation[1]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Y, control_items.orientation[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Z, control_items.orientation[3]);
  // Items to check status of motors
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_L, control_items.present_position[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_R, control_items.present_position[MortorLocation::RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_L, control_items.present_velocity[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_R, control_items.present_velocity[MortorLocation::RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_L, control_items.present_current[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_R, control_items.present_current[MortorLocation::RIGHT]);
  // Items to control motors
  dxl_slave.addControlItem(ADDR_MOTOR_CONNECT, control_items.is_connect_motors);
  dxl_slave.addControlItem(ADDR_MOTOR_TORQUE, control_items.motor_torque_enable_state);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_X, control_items.cmd_vel_linear[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Y, control_items.cmd_vel_linear[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Z, control_items.cmd_vel_linear[2]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_X, control_items.cmd_vel_angular[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Y, control_items.cmd_vel_angular[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Z, control_items.cmd_vel_angular[2]);  
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_L, control_items.profile_acceleration[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_R, control_items.profile_acceleration[MortorLocation::RIGHT]);

  dxl_slave.addControlItem(ADDR_OPERATING_MODE, control_items.operating_mode);
  dxl_slave.addControlItem(ADDR_CMD_PWM_L, control_items.cmd_pwm[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_CMD_PWM_R, control_items.cmd_pwm[MortorLocation::RIGHT]);

  dxl_slave.addControlItem(ADDR_SEGWAY_CTRL_ENABLE, control_items.segway_ctrl_enable);
  dxl_slave.addControlItem(ADDR_SEGWAY_K_EXT_1, control_items.segway_k_ext[0]);
  dxl_slave.addControlItem(ADDR_SEGWAY_K_EXT_2, control_items.segway_k_ext[1]);
  dxl_slave.addControlItem(ADDR_SEGWAY_K_EXT_3, control_items.segway_k_ext[2]);
  dxl_slave.addControlItem(ADDR_SEGWAY_K_EXT_4, control_items.segway_k_ext[3]);

  if (p_tb3_model_info->has_manipulator == true) {
    control_items.joint_goal_position_wr_joint = false;
    control_items.joint_goal_position_wr_gripper = false;
    control_items.joint_goal_position_rd = false;
    control_items.joint_profile_acc_wr_joint = false;
    control_items.joint_profile_acc_wr_gripper = false;
    control_items.joint_profile_acc_rd = false;
    control_items.joint_profile_vel_wr_joint = false;
    control_items.joint_profile_vel_wr_gripper = false;
    control_items.joint_profile_vel_rd = false;
    control_items.joint_goal_current_wr_joint = false;
    control_items.joint_goal_current_wr_gripper = false;
    control_items.joint_goal_current_rd = false;

    // Items to joint motors
    dxl_slave.addControlItem(ADDR_TORQUE_JOINT, control_items.joint_torque_enable_state);

    dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_1, control_items.joint_goal_position.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_2, control_items.joint_goal_position.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_3, control_items.joint_goal_position.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_4, control_items.joint_goal_position.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_GRIPPER, control_items.joint_goal_position.value[GRIPPER]);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_WR_JOINT, control_items.joint_goal_position_wr_joint);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_WR_GRIPPER, control_items.joint_goal_position_wr_gripper);
    dxl_slave.addControlItem(ADDR_GOAL_POSITION_RD, control_items.joint_goal_position_rd);

    dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_1, control_items.joint_present_position.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_2, control_items.joint_present_position.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_3, control_items.joint_present_position.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_4, control_items.joint_present_position.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_PRESENT_POSITION_GRIPPER, control_items.joint_present_position.value[GRIPPER]);

    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_1, control_items.joint_present_velocity.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_2, control_items.joint_present_velocity.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_3, control_items.joint_present_velocity.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_4, control_items.joint_present_velocity.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_GRIPPER, control_items.joint_present_velocity.value[GRIPPER]);

    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_1, control_items.joint_present_current.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_2, control_items.joint_present_current.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_3, control_items.joint_present_current.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_4, control_items.joint_present_current.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_GRIPPER, control_items.joint_present_current.value[GRIPPER]);

    dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_1, control_items.joint_profile_acc.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_2, control_items.joint_profile_acc.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_3, control_items.joint_profile_acc.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_4, control_items.joint_profile_acc.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_GRIPPER, control_items.joint_profile_acc.value[GRIPPER]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_WR_JOINT, control_items.joint_profile_acc_wr_joint);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_WR_GRIPPER, control_items.joint_profile_acc_wr_gripper);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC_RD, control_items.joint_profile_acc_rd);

    dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_1, control_items.joint_profile_vel.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_2, control_items.joint_profile_vel.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_3, control_items.joint_profile_vel.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_4, control_items.joint_profile_vel.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_GRIPPER, control_items.joint_profile_vel.value[GRIPPER]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_WR_JOINT, control_items.joint_profile_vel_wr_joint);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_WR_GRIPPER, control_items.joint_profile_vel_wr_gripper);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL_RD, control_items.joint_profile_vel_rd);

    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_1, control_items.joint_goal_current.value[JOINT_1]);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_2, control_items.joint_goal_current.value[JOINT_2]);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_3, control_items.joint_goal_current.value[JOINT_3]);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_4, control_items.joint_goal_current.value[JOINT_4]);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_GRIPPER, control_items.joint_goal_current.value[GRIPPER]);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_WR_JOINT, control_items.joint_goal_current_wr_joint);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_WR_GRIPPER, control_items.joint_goal_current_wr_gripper);
    dxl_slave.addControlItem(ADDR_GOAL_CURRENT_RD, control_items.joint_goal_current_rd);    
  }

  // Set user callback function for processing write command from master.
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  // Check connection state with motors.
  if(motor_driver.is_connected() == true){
    motor_driver.set_operating_mode(control_items.operating_mode);
    motor_driver.set_torque(true);
    control_items.device_status = STATUS_RUNNING;
    set_connection_state_with_motors(true);
    DEBUG_PRINTLN("Wheel motors are connected");
  }else{
    control_items.device_status = STATUS_NOT_CONNECTED_MOTORS;
    set_connection_state_with_motors(false);
    DEBUG_PRINTLN("Can't communicate with the motor!");
    DEBUG_PRINTLN("  Please check the connection to the motor and the power supply.");
    DEBUG_PRINTLN();
  } 
  control_items.is_connect_motors = get_connection_state_with_motors();  

  if (p_tb3_model_info->has_manipulator == true) {
    // Check connection state with joints.
    if(manipulator_driver.is_connected() == true){
      manipulator_driver.set_torque(true);    
      control_items.is_connect_manipulator = true;
      set_connection_state_with_joints(true);
      DEBUG_PRINTLN("Joint motors are connected");      
    }else{
      control_items.is_connect_manipulator = false;
      set_connection_state_with_joints(false);
      DEBUG_PRINTLN("Can't communicate with the joint!");
      DEBUG_PRINTLN("  Please check the connection to the joint motor and the power supply.");
      DEBUG_PRINTLN();
    } 
  }

  // Init IMU 
  sensors.initIMU();
  sensors.calibrationGyro();

  // Init Segway controller
  init_segway_controller();

  //To indicate that the initialization is complete.
  sensors.makeMelody(1); 

  DEBUG_PRINTLN("Begin End...");
}

/*******************************************************************************
* Definition for TurtleBot3Core 'run()' function
*******************************************************************************/
void TurtleBot3Core::run()
{
  static uint32_t pre_time_to_control_motor;

  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node();

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);
  // Check push button pressed for simple test drive
  test_motors_with_buttons(diagnosis.getButtonPress(3000));

  /* For sensing and run buzzer */
  // Update the IMU unit
  sensors.updateIMU();
  // Update sonar data
  // TODO: sensors.updateSonar(t);
  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For getting command from rc100 */
  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100);

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_imu(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_times(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_gpios(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_motor_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_battery_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_analog_sensors(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_joint_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();

  /* For controlling DYNAMIXEL motors (Wheels) */  
  if (millis()-pre_time_to_control_motor >= INTERVAL_MS_TO_CONTROL_MOTOR)
  {
    pre_time_to_control_motor = millis();
    if(get_connection_state_with_ros2_node() == false){
      memset(goal_velocity_from_cmd, 0, sizeof(goal_velocity_from_cmd));
    }
    if(get_connection_state_with_motors() == true){
      if(control_items.operating_mode == OperatingMode::OP_PWM){
        if (control_items.segway_ctrl_enable == true)
          update_goal_pwm_from_segway_controller();
        else
          update_goal_pwm_from_2values();
        motor_driver.write_pwm(goal_pwm[MortorLocation::LEFT], goal_pwm[MortorLocation::RIGHT]);
      }else{
        update_goal_velocity_from_3values();
        motor_driver.control_motors(p_tb3_model_info->wheel_separation, goal_velocity[VelocityType::LINEAR], goal_velocity[VelocityType::ANGULAR]);
      }
    }
  }  
}


/*******************************************************************************
* Function definition for updating velocity values 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void update_goal_velocity_from_3values(void)
{
  goal_velocity[VelocityType::LINEAR]  = goal_velocity_from_button[VelocityType::LINEAR]  + goal_velocity_from_cmd[VelocityType::LINEAR]  + goal_velocity_from_rc100[VelocityType::LINEAR];
  goal_velocity[VelocityType::ANGULAR] = goal_velocity_from_button[VelocityType::ANGULAR] + goal_velocity_from_cmd[VelocityType::ANGULAR] + goal_velocity_from_rc100[VelocityType::ANGULAR];

  sensors.setLedPattern(goal_velocity[VelocityType::LINEAR], goal_velocity[VelocityType::ANGULAR]);
}


/*******************************************************************************
* Function definition for updating pwm values 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void update_goal_pwm_from_2values(void)
{
  goal_pwm[MortorLocation::LEFT]  = goal_pwm_from_cmd[MortorLocation::LEFT]  + goal_pwm_from_button[MortorLocation::LEFT];
  goal_pwm[MortorLocation::RIGHT] = goal_pwm_from_cmd[MortorLocation::RIGHT] + goal_pwm_from_button[MortorLocation::RIGHT];
}


/*******************************************************************************
* Function definition for initializing onboard segway controller 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void init_segway_controller(void)
{
  const float omega_c = 2.0f * PI * segway_f_grenz;
  const float k = omega_c * segway_ts;
  const float den = 2.0f + k;

  segway_lp_b0 = k / den;
  segway_lp_b1 = k / den;
  segway_lp_a1 = -(2.0f - k) / den;
  segway_lp_x_prev = 0.0f;
  segway_lp_y_prev = 0.0f;
  segway_x_err = 0.0f;
}


/*******************************************************************************
* Function definition for updating pwm values with onboard segway controller 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void update_goal_pwm_from_segway_controller(void)
{
  // Get phi and phi_dot from IMU
  const float w = control_items.orientation[0];
  const float x = control_items.orientation[1];
  const float y = control_items.orientation[2];
  const float z = control_items.orientation[3];
  const float sin_pitch = constrain(2.0f * (w * y - z * x), -1.0f, 1.0f);
  const float phi = asinf(sin_pitch);
  const float phi_dot = control_items.angular_vel[1];

  // Get s_dot from wheel velocity
  const float present_vel_l = (float)control_items.present_velocity[MortorLocation::LEFT];
  const float present_vel_r = (float)control_items.present_velocity[MortorLocation::RIGHT];
  const float present_vel_avg = 0.5f * (present_vel_l + present_vel_r);
  const float wheel_to_mps = 0.229f * (2.0f * PI / 60.0f) * p_tb3_model_info->wheel_radius;
  const float s_dot_raw = present_vel_avg * wheel_to_mps;
  // Filter s_dot with low-pass filter
  const float s_dot = segway_lp_b0 * s_dot_raw + segway_lp_b1 * segway_lp_x_prev - segway_lp_a1 * segway_lp_y_prev;
  segway_lp_x_prev = s_dot_raw;
  segway_lp_y_prev = s_dot;

  // Calibration for trim values of phi and s_dot
  if (segway_enable_auto_trim == true && segway_calib_done == false) {
    const bool stable_pose = (fabsf(phi) < segway_calib_phi_window_rad);
    // const bool stable_vel = (fabsf(s_dot) < segway_calib_s_dot_window_mps);

    if (stable_pose == true /*&& stable_vel == true*/) {
      segway_calib_samples++;
      segway_calib_sum_phi += phi;
      segway_calib_sum_s_dot += s_dot;

      if (segway_calib_samples >= segway_calib_samples_target) {
        const float inv_n = 1.0f / (float)segway_calib_samples;
        segway_phi_trim_rad = segway_calib_sum_phi * inv_n;
        segway_s_dot_bias_mps = segway_calib_sum_s_dot * inv_n;
        segway_calib_done = true;
        DEBUG_PRINT("Segway trim ready. phi_trim[deg]=");
        DEBUG_PRINT(segway_phi_trim_rad * 180.0f / PI);
        DEBUG_PRINT(", s_dot_bias[m/s]=");
        DEBUG_PRINTLN(segway_s_dot_bias_mps);
        sensors.makeMelody(1); 
      }
    } else {
      segway_calib_samples = 0;
      segway_calib_sum_phi = 0.0f;
      segway_calib_sum_s_dot = 0.0f;
    }

    if (segway_calib_wait_printed == false) {
      DEBUG_PRINTLN("Segway trim: waiting for stable upright pose...");
      segway_calib_wait_printed = true;
    }

    segway_x_err = 0.0f;
    goal_pwm[MortorLocation::LEFT] = 0;
    goal_pwm[MortorLocation::RIGHT] = 0;
    return;
  }
  const float s_dot_cal = s_dot /* - segway_s_dot_bias_mps */;
  const float phi_cal = phi - segway_phi_trim_rad;

  // Check if phi is out of range for safety
  if (fabsf(phi_cal) > segway_phi_max_rad) {
    if (segway_active == true) {
      DEBUG_PRINTLN("Segway: phi out of range, PWM -> 0");
    }
    segway_active = false;
    segway_x_err = 0.0f;
    goal_pwm[MortorLocation::LEFT] = 0;
    goal_pwm[MortorLocation::RIGHT] = 0;
    return;
  }
  segway_active = true;

  // Dynamischer Sollwert aus ROS2 cmd_vel: linear.x [m/s], angular.z [rad/s]
  const float s_dot_ref_dyn = goal_velocity_from_cmd[VelocityType::LINEAR];
  const float ang_vel_cmd   = goal_velocity_from_cmd[VelocityType::ANGULAR];

  // Integrator for s_dot error
  if (segway_enable_integrator == true) {
    segway_x_err += (s_dot_cal - s_dot_ref_dyn) * segway_ts;
    segway_x_err = constrain(segway_x_err, -segway_x_err_max, segway_x_err_max);
  } else {
    segway_x_err = 0.0f;
  }

  float u = -(  control_items.segway_k_ext[0] * s_dot_cal 
              + control_items.segway_k_ext[1] * phi_cal 
              + control_items.segway_k_ext[2] * phi_dot 
              + control_items.segway_k_ext[3] * segway_x_err);
  u = constrain(u, -segway_u_max, segway_u_max);

  int16_t pwm_bal = 0;
  if (fabsf(segway_u_max) > 1e-6f) {
    pwm_bal = (int16_t)roundf((u / segway_u_max) * (float)max_pwm_value);
  }
  pwm_bal = constrain(pwm_bal, min_pwm_value, max_pwm_value);

    const int16_t pwm_turn = (int16_t)constrain(roundf(ang_vel_cmd * segway_turn_gain), (float)min_pwm_value, (float)max_pwm_value);

  goal_pwm[MortorLocation::LEFT]  = (int16_t)constrain((float)(pwm_bal - pwm_turn), (float)min_pwm_value, (float)max_pwm_value);
  goal_pwm[MortorLocation::RIGHT] = (int16_t)constrain((float)(pwm_bal + pwm_turn), (float)min_pwm_value, (float)max_pwm_value);
}


/*******************************************************************************
* Function definition for updating control items in TB3.
*******************************************************************************/
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_times(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.dev_time_millis = millis();
    control_items.dev_time_micros = micros();
  } 
}

void update_gpios(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.user_led[0] = digitalRead(BDPIN_GPIO_4);
    control_items.user_led[1] = digitalRead(BDPIN_GPIO_6);
    control_items.user_led[2] = digitalRead(BDPIN_GPIO_8);
    control_items.user_led[3] = digitalRead(BDPIN_GPIO_10);

    control_items.push_button[0] = digitalRead(BDPIN_PUSH_SW_1);
    control_items.push_button[1] = digitalRead(BDPIN_PUSH_SW_2);

    control_items.bumper[0] = sensors.getBumper1State();
    control_items.bumper[1] = sensors.getBumper2State();
  }  
}

void update_battery_status(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float bat_voltage, bat_percent;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    bat_voltage = sensors.checkVoltage();
    control_items.bat_voltage_x100 = (uint32_t)(bat_voltage*100);

    if(bat_voltage >= 3.5*3){
      bat_percent = map_float(bat_voltage, 3.5*3, 4.1*3, 0.0, 100.0);
      control_items.bat_percent_x100 = (uint32_t)(bat_percent*100);
    }
  }
}

void update_analog_sensors(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.illumination = (uint16_t)sensors.getIlluminationData();
    control_items.ir_sensor = (uint32_t)sensors.getIRsensorData();
    control_items.sornar = (float)sensors.getSonarData();
  }
}

void update_imu(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float* p_imu_data;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    p_imu_data = sensors.getImuAngularVelocity();
    memcpy(control_items.angular_vel, p_imu_data, sizeof(control_items.angular_vel));

    p_imu_data = sensors.getImuLinearAcc();
    memcpy(control_items.linear_acc, p_imu_data, sizeof(control_items.linear_acc));

    p_imu_data = sensors.getImuMagnetic();
    memcpy(control_items.magnetic, p_imu_data, sizeof(control_items.magnetic));

    p_imu_data = sensors.getOrientation();
    memcpy(control_items.orientation, p_imu_data, sizeof(control_items.orientation));
  }  
}

void update_motor_status(uint32_t interval_ms)
{
  static uint32_t pre_time;
  int16_t current_l, current_r;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();


    uint32_t pre_time_dxl;

    pre_time_dxl = millis();
    if(get_connection_state_with_motors() == true){
      motor_driver.read_present_position(control_items.present_position[MortorLocation::LEFT], control_items.present_position[MortorLocation::RIGHT]);
      motor_driver.read_present_velocity(control_items.present_velocity[MortorLocation::LEFT], control_items.present_velocity[MortorLocation::RIGHT]);
      if(motor_driver.read_present_current(current_l, current_r) == true){
        control_items.present_current[MortorLocation::LEFT] = current_l;
        control_items.present_current[MortorLocation::RIGHT] = current_r;
      }

      control_items.motor_torque_enable_state = motor_driver.get_torque();
    }
  }  
}

void update_joint_status(uint32_t interval_ms)
{
  static uint32_t pre_time;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    manipulator_driver.read_present_position(control_items.joint_present_position);
    manipulator_driver.read_present_velocity(control_items.joint_present_velocity);
    manipulator_driver.read_present_current(control_items.joint_present_current);

    if(get_connection_state_with_joints() == true){

      control_items.joint_torque_enable_state = manipulator_driver.get_torque();
    }
  }  
}

/*******************************************************************************
* Callback function definition to be used in communication with the ROS2 node.
*******************************************************************************/
static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)arg;

  switch(item_addr)
  {
    case ADDR_MODEL_INFORM:
      control_items.model_inform = p_tb3_model_info->model_info;
      dxl_err_code = DXL_ERR_ACCESS;
      break;

    case ADDR_DEBUG_MODE:
      if (control_items.debug_mode == true)
        DEBUG_PRINTLN("Debug Mode : Enabled");
      else
        DEBUG_PRINTLN("Debug Mode : Disabled");
      break;

    case ADDR_SOUND:
      sensors.makeMelody(control_items.buzzer_sound);
      break;

    case ADDR_IMU_RECALIBRATION:
      if(control_items.imu_recalibration == true){
        sensors.calibrationGyro();
        control_items.imu_recalibration = false;
      }
      break;

    case ADDR_MOTOR_TORQUE:
      if(get_connection_state_with_motors() == true)
        motor_driver.set_torque(control_items.motor_torque_enable_state);
      break;

    case ADDR_OPERATING_MODE:
      if(get_connection_state_with_motors() == true)
        motor_driver.set_operating_mode(control_items.operating_mode);
      break;

    case ADDR_CMD_PWM_L:
      goal_pwm_from_cmd[MortorLocation::LEFT]  = constrain(control_items.cmd_pwm[MortorLocation::LEFT],  min_pwm_value, max_pwm_value);
      break;

    case ADDR_CMD_PWM_R:
      goal_pwm_from_cmd[MortorLocation::RIGHT] = constrain(control_items.cmd_pwm[MortorLocation::RIGHT], min_pwm_value, max_pwm_value);
      break;

    case ADDR_CMD_VEL_LINEAR_X:
      goal_velocity_from_cmd[VelocityType::LINEAR] = constrain((float)(control_items.cmd_vel_linear[0]*0.01f), min_linear_velocity, max_linear_velocity);
      break;

    case ADDR_CMD_VEL_ANGULAR_Z:
      goal_velocity_from_cmd[VelocityType::ANGULAR] = constrain((float)(control_items.cmd_vel_angular[2]*0.01f), min_angular_velocity, max_angular_velocity);
      break;            

    case ADDR_PROFILE_ACC_L:
    case ADDR_PROFILE_ACC_R:
      if(get_connection_state_with_motors() == true)
        motor_driver.write_profile_acceleration(control_items.profile_acceleration[MortorLocation::LEFT], control_items.profile_acceleration[MortorLocation::RIGHT]);
      break;

    case ADDR_SEGWAY_CTRL_ENABLE:
      segway_active = false;
      segway_x_err = 0.0f;
      init_segway_controller();
      segway_calib_done = false;
      segway_calib_wait_printed = false;
      segway_calib_samples = 0;
      segway_calib_sum_phi = 0.0f;
      segway_calib_sum_s_dot = 0.0f;
      segway_s_dot_bias_mps = 0.0f;
      break;

    case ADDR_SEGWAY_K_EXT_1:
    case ADDR_SEGWAY_K_EXT_2:
    case ADDR_SEGWAY_K_EXT_3:
    case ADDR_SEGWAY_K_EXT_4:
      segway_x_err = 0.0f;
      init_segway_controller();
      break;

    case ADDR_TORQUE_JOINT:
      manipulator_driver.set_torque(control_items.joint_torque_enable_state);
      break;

    // ADDR_GOAL_POSITION
    //
    case ADDR_GOAL_POSITION_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_joint == true) {
        manipulator_driver.write_goal_position_joint(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_joint = false;
      break;

    case ADDR_GOAL_POSITION_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_gripper == true) {
        manipulator_driver.write_goal_position_gripper(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_gripper = false;
      break;

    case ADDR_GOAL_POSITION_RD:
      if (control_items.joint_goal_position_rd == true) {
        manipulator_driver.read_goal_position(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_rd = false;
      break;

    // ADDR_PROFILE_ACC
    //
    case ADDR_PROFILE_ACC_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_joint == true) {
        manipulator_driver.write_profile_acceleration_joint(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_joint = false;
      break;      

    case ADDR_PROFILE_ACC_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_gripper == true) {
        manipulator_driver.write_profile_acceleration_gripper(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_joint = false;
      break;      

    case ADDR_PROFILE_ACC_RD:
      if (control_items.joint_profile_acc_rd == true) {
        manipulator_driver.read_profile_acceleration(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_rd = false;
      break;     

    // ADDR_PROFILE_VEL
    //
    case ADDR_PROFILE_VEL_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_joint == true) {
        manipulator_driver.write_profile_velocity_joint(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_joint = false;
      break;      

    case ADDR_PROFILE_VEL_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_gripper == true) {
        manipulator_driver.write_profile_velocity_gripper(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_gripper = false;
      break;   

    case ADDR_PROFILE_VEL_RD:
      if (control_items.joint_profile_vel_rd == true) {
        manipulator_driver.read_profile_velocity(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_rd = false;
      break;      

    // ADDR_GOAL_CURRENT
    //
    case ADDR_GOAL_CURRENT_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_joint == true) {
        manipulator_driver.write_goal_current_joint(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_joint = false;
      break;      

    case ADDR_GOAL_CURRENT_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_gripper == true) {
        manipulator_driver.write_goal_current_gripper(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_gripper = false;
      break;   

    case ADDR_GOAL_CURRENT_RD:
      if (control_items.joint_goal_current_rd == true) {
        manipulator_driver.read_goal_current(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_rd = false;
      break;        
  }
}


/*******************************************************************************
* Function definition to check the connection status with the ROS2 node.
*******************************************************************************/
static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node()
{
  static uint32_t pre_time;
  static uint8_t pre_data;
  static bool pre_state;

  //To wait for IMU Calibration
  if(pre_state != get_connection_state_with_ros2_node()){
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if(pre_data != control_items.heart_beat || control_items.debug_mode == true){
    pre_time = millis();
    pre_data = control_items.heart_beat;
    set_connection_state_with_ros2_node(true);
  }else{
    if(millis()-pre_time >= HEARTBEAT_TIMEOUT_MS){
      pre_time = millis();
      set_connection_state_with_ros2_node(false);
    }
  }

  control_items.is_connect_ros2_node = get_connection_state_with_ros2_node();
}


/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_motors = false;

static bool get_connection_state_with_motors()
{
  return is_connected_motors;
}

static void set_connection_state_with_motors(bool is_connected)
{
  is_connected_motors = is_connected;
}

/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_joints = false;

static bool get_connection_state_with_joints()
{
  return is_connected_joints;
}

static void set_connection_state_with_joints(bool is_connected)
{
  is_connected_joints = is_connected;
}

/*******************************************************************************
* Function definition to test motors using the built-in buttons of OpenCR.
*******************************************************************************/
const float TICK2RAD = 0.001533981; // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
const float TEST_DISTANCE = 0.300; // meter
const float TEST_RADIAN = 3.14; // 180 degree

void test_motors_with_buttons(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  if(get_connection_state_with_motors() == true){
    motor_driver.read_present_position(current_tick[MortorLocation::LEFT], current_tick[MortorLocation::RIGHT]);
  }

  if (buttons & (1<<0))  
  {
    move[VelocityType::LINEAR] = true;
    saved_tick[MortorLocation::RIGHT] = current_tick[MortorLocation::RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[VelocityType::ANGULAR] = true;
    saved_tick[MortorLocation::RIGHT] = current_tick[MortorLocation::RIGHT];

    diff_encoder = (TEST_RADIAN * p_tb3_model_info->turning_radius) / (0.207 / 4096);
  }

  if (move[VelocityType::LINEAR])
  {    
    if (abs(saved_tick[MortorLocation::RIGHT] - current_tick[MortorLocation::RIGHT]) <= diff_encoder)
    {
      if (control_items.operating_mode == OperatingMode::OP_PWM){
        goal_pwm_from_button[MortorLocation::LEFT]  = 300;
        goal_pwm_from_button[MortorLocation::RIGHT] = 300;
      }else{
      goal_velocity_from_button[VelocityType::LINEAR]  = 0.05;
      }
    }
    else
    {
      if (control_items.operating_mode == OperatingMode::OP_PWM){
        goal_pwm_from_button[MortorLocation::LEFT]  = 0;
        goal_pwm_from_button[MortorLocation::RIGHT] = 0;
      }else{
        goal_velocity_from_button[VelocityType::LINEAR]  = 0.0;
      }
      move[VelocityType::LINEAR] = false;
    }
  }
  else if (move[VelocityType::ANGULAR])
  {   
    if (abs(saved_tick[MortorLocation::RIGHT] - current_tick[MortorLocation::RIGHT]) <= diff_encoder)
    {
      if (control_items.operating_mode == OperatingMode::OP_PWM){
        goal_pwm_from_button[MortorLocation::LEFT]  = 300;
        goal_pwm_from_button[MortorLocation::RIGHT] = -300;
      }else{
        goal_velocity_from_button[VelocityType::ANGULAR]= -0.7;
      }
    }
    else
    {
      if (control_items.operating_mode == OperatingMode::OP_PWM){
        goal_pwm_from_button[MortorLocation::LEFT]  = 0;
        goal_pwm_from_button[MortorLocation::RIGHT] = 0;
      }else{
        goal_velocity_from_button[VelocityType::ANGULAR]= 0.0;
      }
      move[VelocityType::ANGULAR] = false;
    }
  }
}