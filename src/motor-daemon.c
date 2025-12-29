#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <pthread.h>

#include <json_config.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>

// Configuration structures
#define MOTOR_GPIO_STR_LEN 64
#define MOTOR_POS_STR_LEN 32

typedef struct {
  int max_steps; // maximum steps for axis
  int home;      // center/home position
  int speed;     // max/default speed for axis
  int timeout;   // seconds
} AxisCfg;

typedef struct {
  bool gpio_invert;
  bool homing;
  bool is_spi;
  int gpio_power;
  int gpio_switch;
  char gpio_pan[MOTOR_GPIO_STR_LEN];
  char gpio_tilt[MOTOR_GPIO_STR_LEN];
  char pos0[MOTOR_POS_STR_LEN];
} MotorHwCfg;

typedef struct {
  int loglevel; // 0=DEBUG,1=INFO (simple mapping)
  AxisCfg pan;  // X axis
  AxisCfg tilt; // Y axis
  MotorHwCfg hw;
  bool loaded; // whether configuration was loaded
} MotorConfig;

static MotorConfig g_cfg = {
    .loglevel = 0,
    .pan = {0, 0, 0, 0},
    .tilt = {0, 0, 0, 0},
    .hw = {0},
    .loaded = false,
};

static int parse_loglevel(const char *s) {
  if (!s)
    return 0; // default DEBUG
  if (strcasecmp(s, "INFO") == 0)
    return 1;
  if (strcasecmp(s, "DEBUG") == 0)
    return 0;
  return 0;
}

static int json_get_int_jct(JsonValue *obj, const char *key, int *out) {
  if (!obj || obj->type != JSON_OBJECT || !key || !out)
    return 0;

  JsonValue *value = get_object_item(obj, key);
  if (!value)
    return 0;

  if (value->type == JSON_NUMBER) {
    *out = (int)value->value.number;
    return 1;
  }

  if (value->type == JSON_STRING && value->value.string &&
      value->value.string[0] != '\0') {
    char *endptr = NULL;
    long parsed = strtol(value->value.string, &endptr, 10);
    if (endptr && *endptr == '\0') {
      *out = (int)parsed;
      return 1;
    }
  }

  return 0;
}

static const char *json_get_string_jct(JsonValue *obj, const char *key) {
  if (!obj || obj->type != JSON_OBJECT || !key)
    return NULL;

  JsonValue *value = get_object_item(obj, key);
  if (value && value->type == JSON_STRING)
    return value->value.string;

  return NULL;
}

static int json_get_bool_jct(JsonValue *obj, const char *key, bool *out) {
  if (!obj || obj->type != JSON_OBJECT || !key || !out)
    return 0;

  JsonValue *value = get_object_item(obj, key);
  if (!value)
    return 0;

  if (value->type == JSON_BOOL) {
    *out = value->value.boolean;
    return 1;
  }

  if (value->type == JSON_NUMBER) {
    *out = (value->value.number != 0);
    return 1;
  }

  if (value->type == JSON_STRING && value->value.string &&
      value->value.string[0] != '\0') {
    const char *s = value->value.string;
    if (strcasecmp(s, "true") == 0 || strcasecmp(s, "yes") == 0 ||
        strcasecmp(s, "on") == 0 || strcmp(s, "1") == 0) {
      *out = true;
      return 1;
    }
    if (strcasecmp(s, "false") == 0 || strcasecmp(s, "no") == 0 ||
        strcasecmp(s, "off") == 0 || strcmp(s, "0") == 0) {
      *out = false;
      return 1;
    }
  }

  return 0;
}

static bool parse_pos0_string(const char *pos0, int *pan_home, int *tilt_home) {
  if (!pos0 || !pan_home || !tilt_home)
    return false;

  char *endptr = NULL;
  long pan = strtol(pos0, &endptr, 10);
  if (endptr == pos0)
    return false;

  while (*endptr != '\0' && isspace((unsigned char)*endptr))
    ++endptr;

  if (*endptr != ',')
    return false;
  ++endptr;

  while (*endptr != '\0' && isspace((unsigned char)*endptr))
    ++endptr;

  if (*endptr == '\0')
    return false;

  char *tilt_end = NULL;
  long tilt = strtol(endptr, &tilt_end, 10);
  if (tilt_end == endptr)
    return false;

  while (*tilt_end != '\0') {
    if (!isspace((unsigned char)*tilt_end))
      return false;
    ++tilt_end;
  }

  *pan_home = (int)pan;
  *tilt_home = (int)tilt;
  return true;
}

static void sanitize_axis_cfg(AxisCfg *axis) {
  if (!axis)
    return;

  if (axis->max_steps < 0)
    axis->max_steps = 0;
  if (axis->speed < 0)
    axis->speed = 0;
  if (axis->timeout < 0)
    axis->timeout = 0;
  if (axis->home < 0)
    axis->home = 0;
  if (axis->max_steps > 0 && axis->home > axis->max_steps)
    axis->home = axis->max_steps / 2;
}

static void reset_config_defaults(void) {
  g_cfg.loglevel = 0;
  g_cfg.pan = (AxisCfg){0};
  g_cfg.tilt = (AxisCfg){0};
  g_cfg.hw = (MotorHwCfg){0};
  g_cfg.hw.gpio_switch = -1;
  g_cfg.hw.gpio_power = -1;
  g_cfg.loaded = false;
}

static bool parse_modern_layout(JsonValue *root, JsonValue *motors) {
  if (!motors || motors->type != JSON_OBJECT)
    return false;

  bool parsed = false;

  const char *loglevel = json_get_string_jct(motors, "loglevel");
  if (!loglevel)
    loglevel = json_get_string_jct(root, "loglevel");
  if (loglevel) {
    g_cfg.loglevel = parse_loglevel(loglevel);
    parsed = true;
  }

  bool bool_value = false;
  if (json_get_bool_jct(motors, "gpio_invert", &bool_value)) {
    g_cfg.hw.gpio_invert = bool_value;
    parsed = true;
  }
  if (json_get_bool_jct(motors, "homing", &bool_value)) {
    g_cfg.hw.homing = bool_value;
    parsed = true;
  }
  if (json_get_bool_jct(motors, "is_spi", &bool_value)) {
    g_cfg.hw.is_spi = bool_value;
    parsed = true;
  }

  parsed |= json_get_int_jct(motors, "gpio_power", &g_cfg.hw.gpio_power);
  parsed |= json_get_int_jct(motors, "gpio_switch", &g_cfg.hw.gpio_switch);

  const char *gpio_pan = json_get_string_jct(motors, "gpio_pan");
  if (gpio_pan) {
    strncpy(g_cfg.hw.gpio_pan, gpio_pan, sizeof(g_cfg.hw.gpio_pan) - 1);
    g_cfg.hw.gpio_pan[sizeof(g_cfg.hw.gpio_pan) - 1] = '\0';
    parsed = true;
  }

  const char *gpio_tilt = json_get_string_jct(motors, "gpio_tilt");
  if (gpio_tilt) {
    strncpy(g_cfg.hw.gpio_tilt, gpio_tilt, sizeof(g_cfg.hw.gpio_tilt) - 1);
    g_cfg.hw.gpio_tilt[sizeof(g_cfg.hw.gpio_tilt) - 1] = '\0';
    parsed = true;
  }

  const char *pos0 = json_get_string_jct(motors, "pos_0");
  if (pos0) {
    strncpy(g_cfg.hw.pos0, pos0, sizeof(g_cfg.hw.pos0) - 1);
    g_cfg.hw.pos0[sizeof(g_cfg.hw.pos0) - 1] = '\0';
    if (parse_pos0_string(pos0, &g_cfg.pan.home, &g_cfg.tilt.home))
      parsed = true;
  }

  parsed |= json_get_int_jct(motors, "steps_pan", &g_cfg.pan.max_steps);
  parsed |= json_get_int_jct(motors, "steps_tilt", &g_cfg.tilt.max_steps);
  parsed |= json_get_int_jct(motors, "speed_pan", &g_cfg.pan.speed);
  parsed |= json_get_int_jct(motors, "speed_tilt", &g_cfg.tilt.speed);
  parsed |= json_get_int_jct(motors, "timeout_pan", &g_cfg.pan.timeout);
  parsed |= json_get_int_jct(motors, "timeout_tilt", &g_cfg.tilt.timeout);
  parsed |= json_get_int_jct(motors, "home_pan", &g_cfg.pan.home);
  parsed |= json_get_int_jct(motors, "home_tilt", &g_cfg.tilt.home);

  return parsed;
}

static bool parse_legacy_layout(JsonValue *root) {
  if (!root || root->type != JSON_OBJECT)
    return false;

  bool parsed = false;

  const char *loglevel = json_get_string_jct(root, "loglevel");
  if (loglevel) {
    g_cfg.loglevel = parse_loglevel(loglevel);
    parsed = true;
  }

  JsonValue *pan = get_object_item(root, "pan");
  if (pan && pan->type == JSON_OBJECT) {
    parsed |= json_get_int_jct(pan, "max_steps", &g_cfg.pan.max_steps);
    parsed |= json_get_int_jct(pan, "home", &g_cfg.pan.home);
    parsed |= json_get_int_jct(pan, "speed", &g_cfg.pan.speed);
    parsed |= json_get_int_jct(pan, "timeout", &g_cfg.pan.timeout);
  }

  JsonValue *tilt = get_object_item(root, "tilt");
  if (tilt && tilt->type == JSON_OBJECT) {
    parsed |= json_get_int_jct(tilt, "max_steps", &g_cfg.tilt.max_steps);
    parsed |= json_get_int_jct(tilt, "home", &g_cfg.tilt.home);
    parsed |= json_get_int_jct(tilt, "speed", &g_cfg.tilt.speed);
    parsed |= json_get_int_jct(tilt, "timeout", &g_cfg.tilt.timeout);
  }

  return parsed;
}

static void load_config_file(void) {
  JsonValue *root = parse_json_file("/etc/motors.json");
  reset_config_defaults();
  if (!root) {
    syslog(LOG_DEBUG, "No config file found; using defaults");
    return;
  }

  if (root->type != JSON_OBJECT) {
    syslog(LOG_DEBUG, "Config file root is not a JSON object; ignoring");
    free_json_value(root);
    return;
  }

  bool parsed = false;
  JsonValue *motors = get_object_item(root, "motors");
  if (motors && motors->type == JSON_OBJECT)
    parsed = parse_modern_layout(root, motors);

  if (!parsed)
    parsed = parse_legacy_layout(root);

  if (parsed) {
    sanitize_axis_cfg(&g_cfg.pan);
    sanitize_axis_cfg(&g_cfg.tilt);
    g_cfg.loaded = true;
  } else {
    syslog(LOG_DEBUG, "Config file missing required keys; using defaults");
  }

  free_json_value(root);
}

#define SV_SOCK_PATH "/dev/md"
#define MAX_CONN 5
#define MOTOR_MOVE_STOP 0x0
#define MOTOR_MOVE_RUN 0x1

/* directional_attr */
#define MOTOR_DIRECTIONAL_UP 0x0
#define MOTOR_DIRECTIONAL_DOWN 0x1
#define MOTOR_DIRECTIONAL_LEFT 0x2
#define MOTOR_DIRECTIONAL_RIGHT 0x3

#define MOTOR1_MAX_SPEED 2000
#define MOTOR1_MIN_SPEED 1

/* ioctl cmd */
#define MOTOR_STOP 0x1
#define MOTOR_RESET 0x2
#define MOTOR_MOVE 0x3
#define MOTOR_GET_STATUS 0x4
#define MOTOR_SPEED 0x5
#define MOTOR_GOBACK 0x6
#define MOTOR_CRUISE 0x7
#define MOTOR_SPEED_AXIS 0x8

#define MOTOR_ACTIVE_FLAG "/run/motors-active"
#define PID_SIZE 32

enum motor_status {
  MOTOR_IS_STOP,
  MOTOR_IS_RUNNING,
};

enum motor_inversion {
  MOTOR_NO_INVERSION = 0x0, // No inversion
  MOTOR_INVERT_X = 0x1,     // Invert X only
  MOTOR_INVERT_Y = 0x2,     // Invert Y only
  MOTOR_INVERT_BOTH = 0x3   // Invert both X and Y
};

enum motor_inversion motor_inversion_state =
    MOTOR_NO_INVERSION; // Default is no inversion

struct request {
  char command; // d,r,s,p,b,S,i,j (move, reset,set speed,get position, is
                // busy,Status,initial,JSON)
  char type;    // g,h,c,s (absolute,relative,cruise,stop)
  int x;
  int got_x;
  int y;
  int got_y;
  int speed; // Add speed to the request structure
};

struct motor_status_st {
  int directional_attr;
  int total_steps;
  int current_steps;
  int min_speed;
  int cur_speed;
  int max_speed;
  int move_is_min;
  int move_is_max;
};

struct motor_message {
  int x;
  int y;
  enum motor_status status;
  int speed;
  /* these two members are not standard from the original kernel module */
  unsigned int x_max_steps;
  unsigned int y_max_steps;
  unsigned int inversion_state; // Report the inversion state
};

struct motors_steps {
  int x;
  int y;
};

struct motor_reset_data {
  unsigned int x_max_steps;
  unsigned int y_max_steps;
  unsigned int x_cur_step;
  unsigned int y_cur_step;
};

struct motor_axis_speed {
  int x_speed;
  int y_speed;
};

int motorfd = -1;
struct request request_message; // object for IPC request from client
int last_known_speed = 900;  // Default speed (overridden by config if present)
bool motor_inverted = false; // Global flag for motor inversion

void motor_ioctl(int cmd, void *arg) {
  // basically exists to not pass around the motor FD
  int ret = ioctl(motorfd, cmd, arg);
  if (ret == -1) {
    syslog(LOG_ERR, "ioctl cmd 0x%x failed: %s", cmd, strerror(errno));
  }
}

static void compute_axis_speeds(int requested_speed, int *x_speed, int *y_speed) {
  int xs = requested_speed;
  int ys = requested_speed;

  if (g_cfg.loaded) {
    if (g_cfg.pan.speed > 0 && xs > g_cfg.pan.speed)
      xs = g_cfg.pan.speed;
    if (g_cfg.tilt.speed > 0 && ys > g_cfg.tilt.speed)
      ys = g_cfg.tilt.speed;
  }

  if (x_speed)
    *x_speed = xs;
  if (y_speed)
    *y_speed = ys;
}

static void motor_set_axis_speed(int x_speed, int y_speed) {
  struct motor_axis_speed axis = {
      .x_speed = x_speed,
      .y_speed = y_speed,
  };
  motor_ioctl(MOTOR_SPEED_AXIS, &axis);
}

static int read_uint_sysfs(const char *path, unsigned int *out) {
  FILE *f = fopen(path, "r");
  if (!f)
    return -1;
  char buf[64];
  if (!fgets(buf, sizeof(buf), f)) {
    fclose(f);
    return -1;
  }
  fclose(f);
  unsigned long v = 0;
  char *endp = NULL;
  v = strtoul(buf, &endp, 0);
  if (endp == buf)
    return -1;
  if (out)
    *out = (unsigned int)v;
  return 0;
}

static void motor_get_maxsteps_sysfs(unsigned int *maxx, unsigned int *maxy) {
  unsigned int x = 0, y = 0;
  (void)read_uint_sysfs("/sys/module/motor/parameters/hmaxstep", &x);
  (void)read_uint_sysfs("/sys/module/motor/parameters/vmaxstep", &y);
  if (maxx && x > 0)
    *maxx = x;
  if (maxy && y > 0)
    *maxy = y;
}

void motor_status_get(struct motor_message *msg) {
  motor_ioctl(MOTOR_GET_STATUS, msg);
}

void motor_get_maxsteps(unsigned int *maxx, unsigned int *maxy) {
  struct motor_message msg;
  motor_status_get(&msg);
  if (maxx)
    *maxx = msg.x_max_steps;
  if (maxy)
    *maxy = msg.y_max_steps;
}

int motor_is_busy() {
  struct motor_message msg;
  motor_status_get(&msg);
  return msg.status == MOTOR_IS_RUNNING ? 1 : 0;
}

void motor_steps(int xsteps, int ysteps, int stepspeed) {
  struct motors_steps steps;

  // Apply the correct inversion based on the motor_inversion_state
  steps.x = (motor_inversion_state & MOTOR_INVERT_X) ? -xsteps : xsteps;
  steps.y = (motor_inversion_state & MOTOR_INVERT_Y) ? -ysteps : ysteps;

  syslog(LOG_DEBUG, "Starting relative move");
  int eff_speed_x = stepspeed;
  int eff_speed_y = stepspeed;

  compute_axis_speeds(stepspeed, &eff_speed_x, &eff_speed_y);

  syslog(LOG_DEBUG, " -> steps, X %d (speed %d), Y %d (speed %d)\n",
         steps.x, eff_speed_x, steps.y, eff_speed_y);

  motor_set_axis_speed(eff_speed_x, eff_speed_y);
  motor_ioctl(MOTOR_MOVE, &steps);
  syslog(LOG_DEBUG, "Finished setting relative move");
}

void motor_set_position(int xpos, int ypos, int stepspeed) {
  struct motor_message msg;
  motor_status_get(&msg);

  int deltax = xpos - msg.x;
  int deltay = ypos - msg.y;

  // Apply inversion to deltas based on the inversion state
  if (motor_inversion_state & MOTOR_INVERT_X) {
    deltax = -deltax;
  }
  if (motor_inversion_state & MOTOR_INVERT_Y) {
    deltay = -deltay;
  }

  syslog(LOG_DEBUG, "Starting absolute move");
  int eff_speed = stepspeed;
  syslog(LOG_DEBUG,
         " -> set position current X: %d, Y: %d, steps required X: %d, Y: %d, "
         "speed %d\n",
         msg.x, msg.y, deltax, deltay, eff_speed);
  motor_steps(deltax, deltay, eff_speed);
  syslog(LOG_DEBUG, "Finished setting absolute move");
}

// Poll until motors are idle or timeout (milliseconds). Returns 0 on idle, -1
// on timeout/error
static int wait_until_idle(int timeout_ms, int poll_ms) {
  const int loops = (timeout_ms <= 0 || poll_ms <= 0)
                        ? 1
                        : (timeout_ms + poll_ms - 1) / poll_ms;
  for (int i = 0; i < loops; ++i) {
    struct motor_message msg;
    motor_status_get(&msg);
    if (msg.status == MOTOR_IS_STOP)
      return 0;
    usleep((useconds_t)poll_ms * 1000);
  }
  return -1;
}

// Perform two-phase homing using relative moves and busy polling
// Phase 1: rotate halfway to one side;
// Phase 2: full way back to the opposite side
static int enhanced_homing_daemon(int stepspeed) {
  struct motor_message status;
  motor_status_get(&status);

  unsigned int cfg_x = (g_cfg.loaded && g_cfg.pan.max_steps > 0)
                           ? (unsigned int)g_cfg.pan.max_steps
                           : 0;
  unsigned int cfg_y = (g_cfg.loaded && g_cfg.tilt.max_steps > 0)
                           ? (unsigned int)g_cfg.tilt.max_steps
                           : 0;
  int x_max = cfg_x ? (int)cfg_x : (int)status.x_max_steps;
  int y_max = cfg_y ? (int)cfg_y : (int)status.y_max_steps;
  if (x_max <= 0 || y_max <= 0) {
    unsigned int sfx = 0, sfy = 0;
    motor_get_maxsteps_sysfs(&sfx, &sfy);
    if (x_max <= 0 && sfx > 0)
      x_max = (int)sfx;
    if (y_max <= 0 && sfy > 0)
      y_max = (int)sfy;
  }
  if (x_max <= 0 || y_max <= 0) {
    syslog(LOG_DEBUG,
           "Invalid max steps (x=%d,y=%d); cannot run enhanced homing.", x_max,
           y_max);
    return -1;
  }

  int half_x = x_max / 2;
  int half_y = y_max / 2;
  int full_x = x_max;
  int full_y = y_max;

  int to_s = 0;
  if (g_cfg.loaded) {
    to_s = (g_cfg.pan.timeout > g_cfg.tilt.timeout) ? g_cfg.pan.timeout
                                                    : g_cfg.tilt.timeout;
  }
  int t1_ms = (to_s > 0 ? to_s * 1000 : 10000);
  int t2_ms = (to_s > 0 ? to_s * 1000 : 15000);
  int t3_ms = (to_s > 0 ? to_s * 1000 : 10000);

  // Phase 1: half to one side (negative logical direction)
  syslog(LOG_DEBUG, "Enhanced homing phase 1: x=%d y=%d speed=%d", -half_x,
         -half_y, stepspeed);
  motor_steps(-half_x, -half_y, stepspeed);
  if (wait_until_idle(t1_ms, 100) != 0) {
    syslog(LOG_DEBUG,
           "Timeout waiting for enhanced homing phase 1 to complete.");
    return -1;
  }

  // Phase 2: full way to the opposite side (positive logical direction)
  syslog(LOG_DEBUG, "Enhanced homing phase 2: x=%d y=%d speed=%d", full_x,
         full_y, stepspeed);
  motor_steps(full_x, full_y, stepspeed);
  if (wait_until_idle(t2_ms, 100) != 0) {
    syslog(LOG_DEBUG,
           "Timeout waiting for enhanced homing phase 2 to complete.");
    return -1;
  }

  // Phase 3: move to center position
  int center_x =
      (g_cfg.loaded && g_cfg.pan.home > 0) ? g_cfg.pan.home : (x_max / 2);
  int center_y =
      (g_cfg.loaded && g_cfg.tilt.home > 0) ? g_cfg.tilt.home : (y_max / 2);
  syslog(LOG_DEBUG, "Enhanced homing phase 3 (center): x=%d y=%d speed=%d",
         center_x, center_y, stepspeed);
  motor_set_position(center_x, center_y, stepspeed);
  if (wait_until_idle(t3_ms, 100) != 0) {
    syslog(LOG_DEBUG,
           "Timeout waiting for enhanced homing center move to complete.");
    return -1;
  }

  syslog(LOG_DEBUG, "Enhanced homing completed successfully at center.");
  return 0;
}

int check_pid(char *file_name) {
  FILE *f;
  long pid;
  char pid_buffer[PID_SIZE];

  f = fopen(file_name, "r");
  if (f == NULL)
    return 0;

  if (fgets(pid_buffer, PID_SIZE, f) == NULL) {
    fclose(f);
    return 0;
  }
  fclose(f);

  if (sscanf(pid_buffer, "%ld", &pid) != 1) {
    return 0;
  }

  if (kill(pid, 0) == 0) {
    return 1;
  }

  return 0;
}

int create_pid(char *file_name) {
  FILE *f;
  char pid_buffer[PID_SIZE];

  f = fopen(file_name, "w");
  if (f == NULL)
    return -1;

  memset(pid_buffer, '\0', PID_SIZE);
  sprintf(pid_buffer, "%ld\n", (long)getpid());
  if (fwrite(pid_buffer, strlen(pid_buffer), 1, f) != 1) {
    fclose(f);
    return -2;
  }
  fclose(f);

  return 0;
}

static void write_motion_active_flag() {
  int fd = open(MOTOR_ACTIVE_FLAG, O_CREAT | O_WRONLY | O_TRUNC, 0644);
  if (fd >= 0) {
    const char payload[] = "active\n";
    (void)write(fd, payload, sizeof(payload) - 1);
    close(fd);
  }
}

static void remove_motion_active_flag() { unlink(MOTOR_ACTIVE_FLAG); }

static void *motion_active_tracker(void *arg) {
  (void)arg;
  while (motor_is_busy()) {
    usleep(100 * 1000);
  }
  remove_motion_active_flag();
  return NULL;
}

static void start_motion_active_tracker() {
  write_motion_active_flag();
  pthread_t tid;
  if (pthread_create(&tid, NULL, motion_active_tracker, NULL) == 0) {
    pthread_detach(tid);
  }
}

static void daemonsetup() {
  pid_t pid;

  /* Fork off the parent process */
  pid = fork();

  /* An error occurred */
  if (pid < 0)
    exit(EXIT_FAILURE);

  /* Success: Let the parent terminate */
  if (pid > 0)
    exit(EXIT_SUCCESS);

  /* On success: The child process becomes session leader */
  if (setsid() < 0)
    exit(EXIT_FAILURE);

  /* Catch, ignore and handle signals */
  // TODO: Implement a working signal handler */
  signal(SIGCHLD, SIG_IGN);
  signal(SIGHUP, SIG_IGN);

  /* Fork off for the second time*/
  pid = fork();

  /* An error occurred */
  if (pid < 0)
    exit(EXIT_FAILURE);

  /* Success: Let the parent terminate */
  if (pid > 0)
    exit(EXIT_SUCCESS);

  /* Set new file permissions */
  umask(0);

  /* Change the working directory */
  chdir("/dev/");

  /* Close all open file descriptors */
  int x;
  for (x = sysconf(_SC_OPEN_MAX); x >= 0; x--) {
    close(x);
  }

  /* Open the log file */
  openlog("motors-daemon", LOG_PID, LOG_DAEMON);
}

void requestcleanup() {
  //
  request_message.command = 'd';
  request_message.type = 's';
  request_message.x = 0;
  request_message.got_x = 0;
  request_message.y = 0;
  request_message.got_y = 0;
  request_message.speed = 0; // Reset speed in request
}

int main(int argc, char *argv[]) {
  int c;
  char *pid_file;
  bool skip_reset = false; // Initialize skip_reset to false
  pid_file = "/run/motors-daemon";

  // Load configuration early
  load_config_file();
  if (g_cfg.loaded) {
    int cfg_speed = 0;
    if (g_cfg.pan.speed > 0)
      cfg_speed = g_cfg.pan.speed;
    if (g_cfg.tilt.speed > 0 && g_cfg.tilt.speed > cfg_speed)
      cfg_speed = g_cfg.tilt.speed;
    if (cfg_speed > 0)
      last_known_speed = cfg_speed;
  }

  // setlogmask(LOG_UPTO(LOG_DEBUG));
  while ((c = getopt(argc, argv, "dhp")) != -1) {
    switch (c) {
    case 'd':
      // setlogmask(LOG_UPTO(LOG_DEBUG));
      break;
    case 'p':
      skip_reset = true; // Set skip_reset to true if -p is provided
      break;
    default:
      printf("Usage : \n"
             "\t -d enable debugging messages to syslog\n"
             "\t -h print this help message\n"
             "\t -p skip reset position on launch\n"
             "\t No option to start the daemon\n");
      return EXIT_FAILURE;
      break;
    }
  }
  daemonsetup();
  if (check_pid(pid_file) == 1) {
    syslog(LOG_INFO, "Motors daemon is already running.");
    printf("Motors daemon is already running\n");
    exit(EXIT_FAILURE);
  }
  if (create_pid(pid_file) < 0) {
    syslog(LOG_INFO, "Error creating pid file %s", pid_file);
    exit(EXIT_FAILURE);
  }
  int daemonstop = 0;
  // struct instances
  struct sockaddr_un addr; // socket struct
  struct motor_reset_data motor_reset_data;
  struct motor_message motor_message;

  // acquire control of motor device
  motorfd = open("/dev/motor", 0);

  // Reset/homing on startup, unless skipped via -p
  if (!skip_reset) {
    syslog(LOG_DEBUG, "== Enhanced homing on startup, please wait");
    if (enhanced_homing_daemon(last_known_speed) != 0) {
      syslog(LOG_DEBUG, "Enhanced homing failed at startup, falling back to "
                        "legacy MOTOR_RESET.");
      memset(&motor_reset_data, 0, sizeof(motor_reset_data));
      motor_ioctl(MOTOR_RESET, &motor_reset_data);
    }
  }

  int serverfd = socket(AF_UNIX, SOCK_STREAM, 0);
  syslog(LOG_DEBUG, "Server socket fd = %d", serverfd);
  // check if we could acquire fd for socket
  if (serverfd == -1) {
    syslog(LOG_ERR, "Error initializing the socket, could not get a proper fd");
    closelog();
    exit(EXIT_FAILURE);
  }

  // cleanup for socket path if path already exists
  if (remove(SV_SOCK_PATH) == -1 && errno != ENOENT) {
    syslog(LOG_ERR, "could not remove-%s, exiting", SV_SOCK_PATH);
    closelog();
    exit(EXIT_FAILURE);
  }

  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, SV_SOCK_PATH, sizeof(addr.sun_path) - 1);
  // binding to socket
  if (bind(serverfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) ==
      -1) {
    syslog(LOG_ERR, "Error binding to socket, exiting");
    closelog();
    exit(EXIT_FAILURE);
  }

  // start listening on socket
  if (listen(serverfd, MAX_CONN) == -1) {
    closelog();
    exit(EXIT_FAILURE);
  }

  syslog(LOG_INFO, "motors-daemon started");

  while (daemonstop == 0) {
    // make request object go back to initial value
    syslog(LOG_DEBUG, "Start request cleanup");
    requestcleanup();
    // reset ok to close fd flag (removed unused closeready)
    syslog(LOG_DEBUG, "Waiting to accept a connection");
    // blocking code, wait for a connection
    int clientfd = accept(serverfd, NULL, NULL);
    if (clientfd == -1) {
      syslog(LOG_DEBUG,
             "clientfd is invalid after connection accept, socket doesnt work "
             "and is %i errno : %i",
             clientfd, errno);
      syslog(LOG_DEBUG, "exiting...");
      exit(EXIT_FAILURE);
    }
    syslog(LOG_DEBUG, "Accepting a connection\n");

    // load the message onto the reques_message struct
    if (read(clientfd, &request_message, sizeof(struct request)) == -1) {
      syslog(LOG_DEBUG,
             "Could not read message from motors app, ignore request");
      syslog(LOG_DEBUG, "client fd at this point is %i errno : %i", clientfd,
             errno);
    } else {
      syslog(LOG_DEBUG, "request command is %c", request_message.command);

      if (request_message.speed != 0) {
        last_known_speed = request_message.speed;
        syslog(LOG_DEBUG, "Updating last known speed to %d", last_known_speed);
      } else {
        syslog(LOG_DEBUG, "Using last known speed %d", last_known_speed);
      }

      switch (request_message.command) {
      case 'd': // move direction
        syslog(LOG_DEBUG, "request type is %c", request_message.type);
        switch (request_message.type) {
        case 'g': // relative movement
          motor_steps(request_message.x, request_message.y, last_known_speed);
          syslog(LOG_DEBUG, "request x is %i", request_message.x);
          syslog(LOG_DEBUG, "request y is %i", request_message.y);
          start_motion_active_tracker();
          break;
        case 'h': // absolute movement
          motor_status_get(&motor_message);
          if (request_message.got_x == 0)
            request_message.x =
                motor_message.x; // as we are rewriting initial between requests
                                 // this should not be necessary but leaving as
                                 // is as to not break anything
          if (request_message.got_y == 0)
            request_message.y = motor_message.y;
          motor_set_position(request_message.x, request_message.y,
                             last_known_speed);
          syslog(LOG_DEBUG, "request x is %i", request_message.x);
          syslog(LOG_DEBUG, "request y is %i", request_message.y);
          start_motion_active_tracker();
          break;
        case 'b': // go back
          motor_ioctl(MOTOR_GOBACK, NULL);
          start_motion_active_tracker();
          break;
        case 'c': // cruise
          motor_ioctl(MOTOR_CRUISE, NULL);
          start_motion_active_tracker();
          break;
        case 's': // stop
          motor_ioctl(MOTOR_STOP, NULL);
          remove_motion_active_flag();
          break;
        }
        break;
      case 'r': // reset (homing)
        syslog(LOG_DEBUG, "== Enhanced homing (reset), please wait");
        write_motion_active_flag();
        if (enhanced_homing_daemon(last_known_speed) != 0) {
          syslog(LOG_DEBUG,
                 "Enhanced homing failed, falling back to legacy MOTOR_RESET.");
          memset(&motor_reset_data, 0, sizeof(motor_reset_data));
          motor_ioctl(MOTOR_RESET, &motor_reset_data);
        }
        remove_motion_active_flag();
        break;
      case 'i': // get initial parameters
        // This doesnt seem right, we are returning current information instead
        // of initial parameters not correcting for now, as we want to have
        // functional parity
        motor_status_get(&motor_message);
        syslog(LOG_DEBUG, "Got current status to load into command");
        write(clientfd, &motor_message, sizeof(struct motor_message));
        break;
      case 'j': // get json
        motor_status_get(&motor_message);
        syslog(LOG_DEBUG, "Got current status to load into command");
        write(clientfd, &motor_message, sizeof(struct motor_message));
        break;
      case 'p': // get simple x y position
        motor_status_get(&motor_message);
        syslog(LOG_DEBUG, "Got current status to load into command");
        write(clientfd, &motor_message, sizeof(struct motor_message));

        break;
      case 'b': // is busy
        motor_status_get(&motor_message);
        syslog(LOG_DEBUG, "Got current status to load into command");
        write(clientfd, &motor_message, sizeof(struct motor_message));

        break;
      case 's':                                   // set speed
        last_known_speed = request_message.speed;
        motor_set_axis_speed(last_known_speed, last_known_speed);
        syslog(LOG_DEBUG, "Set speed command, last known speed now %d",
               last_known_speed);
        break;
      case 'I': // Invert motor direction
        switch (request_message.type) {
        case 'x': // Invert X only
          motor_inversion_state ^= MOTOR_INVERT_X;
          syslog(LOG_DEBUG, "Motor inversion X set to %s",
                 (motor_inversion_state & MOTOR_INVERT_X) ? "ON" : "OFF");
          break;
        case 'y': // Invert Y only
          motor_inversion_state ^= MOTOR_INVERT_Y;
          syslog(LOG_DEBUG, "Motor inversion Y set to %s",
                 (motor_inversion_state & MOTOR_INVERT_Y) ? "ON" : "OFF");
          break;
        case 'b': // Invert both X and Y
          motor_inversion_state ^= MOTOR_INVERT_BOTH;
          syslog(LOG_DEBUG, "Motor inversion set to %s",
                 (motor_inversion_state == MOTOR_INVERT_BOTH) ? "BOTH ON"
                                                              : "BOTH OFF");
          break;
        default:
          syslog(LOG_DEBUG, "Invalid inversion command type.");
          break;
        }
        break;
      case 'S': // show status
        motor_status_get(&motor_message);
        motor_message.inversion_state = motor_inversion_state;
        write(clientfd, &motor_message, sizeof(struct motor_message));
        syslog(LOG_DEBUG, "Sent motor status");
        break;
      }

      // need to close fd after each request is completed
      close(clientfd);
    }

    syslog(LOG_DEBUG, "====================");

    // break;
  }

  syslog(LOG_INFO, "motors-daemon terminated.");
  remove_motion_active_flag();
  unlink(pid_file);
  closelog();

  return EXIT_SUCCESS;
}
