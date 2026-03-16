#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <json_config.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>

// Lightweight config for client (useful for default speed)
typedef struct {
  int max_steps;
  int home;
  int speed;
  int timeout;
} AxisCfg;
typedef struct {
  int loglevel;
  AxisCfg pan;
  AxisCfg tilt;
  int loaded;
} ClientCfg;
static ClientCfg c_cfg = {0};

static int json_get_int_jct(JsonValue *obj, const char *key, int *out) {
  if (!obj || obj->type != JSON_OBJECT || !key || !out)
    return 0;

  JsonValue *value = get_object_item(obj, key);
  if (!value)
    return 0;

  if (value->type == JSON_NUMBER) {
    *out = (int)(value->value.number.kind == JSON_NUMBER_INT
                     ? value->value.number.integer
                     : value->value.number.real);
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

static void load_client_config(void) {
  JsonValue *root = parse_json_file("/etc/motors.json");

  c_cfg.pan.speed = 0;
  c_cfg.tilt.speed = 0;
  c_cfg.loaded = 0;

  if (!root)
    return;

  if (root->type == JSON_OBJECT) {
    JsonValue *motors = get_object_item(root, "motors");
    if (motors && motors->type == JSON_OBJECT) {
      json_get_int_jct(motors, "speed_pan", &c_cfg.pan.speed);
      json_get_int_jct(motors, "speed_tilt", &c_cfg.tilt.speed);
    } else {
      JsonValue *pan = get_object_item(root, "pan");
      if (pan && pan->type == JSON_OBJECT)
        json_get_int_jct(pan, "speed", &c_cfg.pan.speed);

      JsonValue *tilt = get_object_item(root, "tilt");
      if (tilt && tilt->type == JSON_OBJECT)
        json_get_int_jct(tilt, "speed", &c_cfg.tilt.speed);
    }

    c_cfg.loaded = 1;
  }

  free_json_value(root);
}

#define SV_SOCK_PATH "/dev/md"
#define BUF_SIZE 15

#define PID_SIZE 32

#define MOTOR_INVERT_X 0x1
#define MOTOR_INVERT_Y 0x2
#define MOTOR_INVERT_BOTH 0x3

enum motor_status {
  MOTOR_IS_STOP,
  MOTOR_IS_RUNNING,
};

struct request {
  char command; // d,r,s,p,b,S,i,j (move, reset,set speed,get position, is
                // busy,Status,initial,JSON)
  char type;    // g,h,c,s (absolute,relative,cruise,stop)
  int x;
  int got_x;
  int y;
  int got_y;
  int speed;           // Add speed to the request structure
  bool speed_supplied; // Track if speed was supplied
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

static void print_json_message(struct motor_message *message,
                               bool include_limits) {
  // One place to maintain JSON shape; values kept as strings for
  // backward-compat
  printf("{");
  printf("\"status\":\"%d\"", message->status);
  printf(",\"xpos\":\"%d\"", message->x);
  printf(",\"ypos\":\"%d\"", message->y);
  if (include_limits) {
    printf(",\"xmax\":\"%d\"", message->x_max_steps);
    printf(",\"ymax\":\"%d\"", message->y_max_steps);
  }
  printf(",\"speed\":\"%d\"", message->speed);
  printf(",\"invert\":\"%d\"", message->inversion_state);
  printf("}\n");
}

void JSON_initial(struct motor_message *message) {
  // initial variant includes xmax/ymax
  print_json_message(message, true);
}

void JSON_status(struct motor_message *message) {
  // status variant omits xmax/ymax
  print_json_message(message, false);
}

void xy_pos(struct motor_message *message) {
  printf("%d,%d\n", (*message).x, (*message).y);
}

void show_status(struct motor_message *message) {
  printf("Max X Steps %d.\n", (*message).x_max_steps);
  printf("Max Y Steps %d.\n", (*message).y_max_steps);
  printf("Status Move: %d.\n", (*message).status);
  printf("X Steps %d.\n", (*message).x);
  printf("Y Steps %d.\n", (*message).y);
  printf("Speed %d.\n", (*message).speed);

  // Report motor inversion status
  if (message->inversion_state == MOTOR_INVERT_BOTH) {
    printf("Motor Inversion: BOTH X and Y are inverted\n");
  } else if (message->inversion_state == MOTOR_INVERT_X) {
    printf("Motor Inversion: X axis is inverted\n");
  } else if (message->inversion_state == MOTOR_INVERT_Y) {
    printf("Motor Inversion: Y axis is inverted\n");
  } else {
    printf("Motor Inversion: OFF\n");
  }
}

int check_daemon(char *file_name) {
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

void print_request_message(struct request *req) {
  printf("Sent message: command=%c, type=%c, x=%d, y=%d, speed=%d, "
         "speed_supplied=%d\n",
         req->command, req->type, req->x, req->y, req->speed,
         req->speed_supplied);
}

void initialize_request_message(struct request *req) {
  memset(req, 0, sizeof(struct request));
  req->command = 'd'; // Default command
  req->type = 's';    // Default type
  req->x = 0;
  req->got_x = 0;
  req->y = 0;
  req->got_y = 0;
  req->speed = 0;
  req->speed_supplied = false;
}

int main(int argc, char *argv[]) {
  char direction = '\0';
  int stepspeed = 900;
  int c;
  char *daemon_pid_file;
  struct request request_message;
  bool verbose = false; // Initialize verbose to false

  initialize_request_message(&request_message);

  // Load client config for defaults and set initial speed if provided
  load_client_config();
  {
    int cfg_speed = 0;
    if (c_cfg.loaded) {
      if (c_cfg.pan.speed > 0 && c_cfg.tilt.speed > 0)
        cfg_speed = (c_cfg.pan.speed < c_cfg.tilt.speed) ? c_cfg.pan.speed
                                                         : c_cfg.tilt.speed;
      else if (c_cfg.pan.speed > 0)
        cfg_speed = c_cfg.pan.speed;
      else if (c_cfg.tilt.speed > 0)
        cfg_speed = c_cfg.tilt.speed;
    }
    if (cfg_speed > 0)
      stepspeed = cfg_speed;
  }

  // openlog ("motors app", LOG_PID, LOG_USER);
  daemon_pid_file = "/var/run/motors-daemon";
  if (check_daemon(daemon_pid_file) == 0) {
    printf("Motors daemon is NOT running, please start the daemon\n");
    exit(EXIT_FAILURE);
  }
  // should open socket here
  struct sockaddr_un addr;

  int serverfd = socket(AF_UNIX, SOCK_STREAM, 0);

  if (serverfd == -1) {
    exit(EXIT_FAILURE);
  }
  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, SV_SOCK_PATH, sizeof(addr.sun_path) - 1);

  // connect to the socket
  if (connect(serverfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) ==
      -1)
    exit(EXIT_FAILURE);

  while ((c = getopt(argc, argv, "d:s:x:y:jipSrvbI:")) != -1) {
    switch (c) {
    case 'd':
      request_message.command = 'd';
      direction = optarg[0];
      break;
    case 's':
      stepspeed = atoi(optarg);
      request_message.speed = stepspeed;
      request_message.speed_supplied =
          true; // Set speed_supplied to true when speed is provided
      request_message.command = 's';
      break;
    case 'x':
      request_message.x = atoi(optarg);
      request_message.got_x = 1;
      break;
    case 'y':
      request_message.y = atoi(optarg);
      request_message.got_y = 1;
      break;
    case 'j':
      request_message.command = 'j';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));

      struct motor_message status;
      read(serverfd, &status, sizeof(struct motor_message));

      JSON_status(&status);
      return 0;
    case 'i':
      // get all initial values
      request_message.command = 'i';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));

      struct motor_message initial;
      read(serverfd, &initial, sizeof(struct motor_message));

      JSON_initial(&initial);
      return 0;
    case 'p':
      request_message.command = 'p';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));

      struct motor_message pos;
      read(serverfd, &pos, sizeof(struct motor_message));

      xy_pos(&pos);
      return 0;
    case 'v':
      verbose = true; // Enable verbose mode
      break;
    case 'r': // reset
      request_message.command = 'r';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));
      return 0;
    case 'S': // status
      request_message.command = 'S';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));

      struct motor_message stat;
      read(serverfd, &stat, sizeof(struct motor_message));

      show_status(&stat);
      return 0;
    case 'I': // Invert motor
      request_message.command = 'I';
      if (optarg) {
        if (strcmp(optarg, "x") == 0) {
          request_message.type = 'x'; // Invert X
        } else if (strcmp(optarg, "y") == 0) {
          request_message.type = 'y'; // Invert Y
        } else if (strcmp(optarg, "b") == 0) {
          request_message.type = 'b'; // Invert both
        } else {
          printf("Invalid option for -I: %s\n", optarg);
          exit(EXIT_FAILURE);
        }
      } else {
        request_message.type = 'b'; // Default to inverting both axes
      }

      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));
      return 0;
    case 'b': // is moving?
      request_message.command = 'b';
      if (verbose)
        print_request_message(&request_message);
      write(serverfd, &request_message, sizeof(struct request));

      struct motor_message busy;
      read(serverfd, &busy, sizeof(struct motor_message));
      if (busy.status == MOTOR_IS_RUNNING) {
        printf("1\n");
        return (1);
      } else {
        printf("0\n");
        return (0);
      }
    default:
      printf("Invalid Argument %c\n", c);
      printf(
          "Usage : %s\n"
          "\t -d Direction step\n"
          "\t -s Speed step (default 900)\n"
          "\t -x X position/step (default 0)\n"
          "\t -y Y position/step (default 0) .\n"
          "\t -r reset to default pos.\n"
          "\t -v verbose mode, prints debugging information while app is "
          "running\n"
          "\t -j return json string xpos,ypos,status.\n"
          "\t -i return json string for all camera parameters\n"
          "\t -p return xpos,ypos as a string\n"
          "\t -b prints 1 if motor is (b)usy moving or 0 if is not\n"
          "\t -S show status\n"
          "\t -I Invert motor direction with 'x', 'y', or 'b' for both axes\n",
          argv[0]);
      exit(EXIT_FAILURE);
    }
  }

  // If the command is speed only, send it and return
  if (request_message.command == 's') {
    if (verbose)
      print_request_message(&request_message);
    write(serverfd, &request_message, sizeof(struct request));
    return 0;
  }

  // Ensure the final request uses the correct speed if supplied
  if (request_message.speed_supplied) {
    request_message.speed = stepspeed;
  } else {
    request_message.speed = 0; // Indicate that speed is not set
  }

  if (request_message.command == 'd') {
    switch (direction) {
    case 's': // stop
      request_message.type = 's';
      break;

    case 'c': // cruise
      request_message.type = 'c';
      break;

    case 'b': // go back
      request_message.type = 'b';
      break;

    case 'h': // set position (absolute movement)
      request_message.type = 'h';
      break;

    case 'g': // move x y (relative movement)
      request_message.type = 'g';
      break;

    default:
      printf("Invalid Direction Argument %c\n", direction);
      printf("Usage : %s -d\n"
             "\t s (Stop)\n"
             "\t c (Cruise)\n"
             "\t b (Go to home position)\n"
             "\t h (Set position X and Y)\n"
             "\t g (Steps X and Y)\n",
             argv[0]);
      exit(EXIT_FAILURE);
    }
  }

  // Print and send the final request message if it's a move command
  if (request_message.command == 'd') {
    if (verbose)
      print_request_message(&request_message);
    write(serverfd, &request_message, sizeof(struct request));
  }

  return 0;
}
