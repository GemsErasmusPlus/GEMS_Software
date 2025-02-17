import peasy.PeasyCam; //<>//

PeasyCam camera;

boolean record;
PGraphics render;
PMatrix mat_scene;
PMatrix mat_A;
PMatrix mat_B;

boolean mat_key = false;
float [] F_B = new float[16];

float fov;
float aspect;
float near;
float far;

boolean[] keys = new boolean[256];


color white = color(249, 246, 238);
color blue = color(70, 130, 180);
color gray = color(200, 200, 200);
color dark_gray = color(100, 100, 100);
color black = color(0, 0, 0);

color ruby = color(255, 87, 51);
color emerald = color(80, 200, 120);
color sapphire = color(15, 82, 186);
color diamond = color(241, 247, 251);

color [] gems_colors = {white, white, sapphire, ruby, emerald, diamond  };
color [] white_colors = {white, white, white, white, white, white};
color [] blue_colors = {blue, blue, blue, blue, blue, blue};
color [] gray_colors = {gray, gray, gray, gray, gray, gray};

float fps = 24.5;
float dt = 1.0/fps;
long previous_time = 0;
long current_time = 0;

enum robot_control_mode {
  human,
    pwm,
    angles
}

Robot robot;
Points points;
int step = 0;

float stroke_width = 5;

void settings() {
  System.setProperty("jogl.disable.openglcore", "true");
  size(800, 600, P3D);
}

void setup()
{
  mat_scene = getMatrix();
  render = createGraphics(4000, 3000, P3D);
  fov = PI / 6;
  aspect = float(render.width) / float(render.height);
  near = 0.1;
  far = 100000;

  camera = new PeasyCam(this, 100, -100, 300, 1500);

  camera.rotateZ(PI/2);
  camera.rotateY(0);
  camera.rotateX(-3*PI/6);

  mat_A = getMatrix();

  frameRate(fps);
  windowTitle("GEMS articulated robot");
  windowResizable(false);

  robot = new Robot(render, 0.0, 0.0);
  points = new Points(robot);
}

void draw()
{
  render.beginDraw();
  {
    mat_B = getMatrix();
    mat_B.get(F_B);
    if (F_B[0]!=1.0) {
      mat_A = mat_B.get();
    }
    render.setMatrix(mat_A);

    render.background(217, 235, 235);
    render.directionalLight(100, 100, 100, 0, 0, -1);
    //render.lights();
    render.ambientLight(170, 170, 170);
    render.perspective(fov, aspect, near, far);

    grid(render, 500, 500, 50, gray);
    render.translate(0, 0, 0);

    current_time = System.nanoTime();
    dt = ((float)(current_time - previous_time)) / 1000000000.0;
    previous_time = current_time;

    robot.update();
    robot.draw();

    points.check();
    points.draw();
  }
  render.endDraw();

  setMatrix(mat_scene);
  background(255, 255, 255);
  image(render, 0, 0, 800, 600);

  textSize(30);
  fill(0, 0, 0);
  text(String.format("A: %.2f rad", robot.angle_A), 60, 60);
  text(String.format("B: %.2f rad", robot.angle_B), 60, 90);

  text(String.format("FPS: %.1f", 1/dt), width - 200, 60);
  text(String.format("Time: %.1f s", robot.time_end), width - 200, 90);

  if (keys['Q']) {
    keys[key] = false;
    render.save("save_hiRes.png");
    saveFrame("save_lowRes.png");
  }
}

void CS() {
  render.strokeWeight(10);

  float x = 100.0;

  render.stroke(255, 0, 0);
  render.line(0, 0, 0, x, 0, 0);

  render.stroke(0, 255, 0);
  render.line(0, 0, 0, 0, x, 0);

  render.stroke(0, 0, 255);
  render.line(0, 0, 0, 0, 0, x);
}

void keyPressed() {
  if (key<255) {
    keys[key] = true;
  }
  if (!robot.run_start) {
    robot.run_start = true;
    robot.time_start = float(millis())/1000;
  }
}

void keyReleased() {
  if (key<255) {
    keys[key] = false;
  }
}

void sphere_draw(float r, color rgb) {
  render.noStroke();
  render.fill(rgb);
  render.sphere(r);
}

class Robot {
  PGraphics render;

  final float effector_length = 500;
  final float offset_vertical = 200;
  final float offset_horizontal = 100;
  final float motor_pwm_max = 1023.0;
  final float rotational_inertia_A = 0.02;
  final float rotational_inertia_B = 0.01;

  float gearbox_ratio = 243.0;
  float gearbox_efficiency = 0.75;

  float torque_motor_max_A = 0.015;
  float torque_motor_max_B = 0.015;

  float torque_A = 0.0;
  float torque_B = 0.0;

  float motor_K_A = -0.015/120;
  float motor_K_B = -0.015/120;

  float motor_pwm_A = 0.0;
  float motor_pwm_B = 0.0;

  float force_A = 0;
  float force_B = 0;

  float angular_speed_A = 0;
  float angular_speed_B = 0;

  float delta_angle_A = 0.0;
  float delta_angle_B = 0.0;

  float angle_A = 0.0;
  float angle_B = 0.0;

  PVector position = new PVector(0, 0, 0);
  PVector new_angles = new PVector(0, 0);

  Boolean run_start = false;
  Boolean run_end = false;
  float time_start = 0;
  float time_end = 0;
  float time_run = 0;
  float time = 0;

  ArrayList<PVector> path = new ArrayList<PVector>();

  robot_control_mode control_mode = robot_control_mode.angles;

  PID_Controller pid_controller_angle_A = new PID_Controller(5.0, 0.0, 1.0);
  PID_Controller pid_controller_angle_B = new PID_Controller(5.0, 0.0, 1.0);


  Robot(PGraphics render, float angle_A, float angle_B) {
    this.render = render;
    this.angle_A = angle_A;
    this.angle_B = angle_B;
  }

  void human_control() {
    if (keys['a']) {
      motor_pwm_A = -1023;
    } else if (keys['s']) {
      motor_pwm_A = 1023;
    } else {
      motor_pwm_A = 0;
    }

    if (keys['e']) {
      motor_pwm_B = 1023;
    } else if (keys['d']) {
      motor_pwm_B = -1023;
    } else {
      motor_pwm_B = 0;
    }
  }

  void pwm_control() {
    if (run_start) {
      if (time_run < 100) {
        motor_pwm_A = 1023;
        motor_pwm_B = 512;
      } else if (time_run < 200) {
        motor_pwm_A = 512;
        motor_pwm_B = 1023;
      } else {
        motor_pwm_A = 0;
        motor_pwm_B = 0;
      }
    }
  }

  void angle_control() {
    float delta_angle_A = new_angles.x - angle_A;
    if (delta_angle_A > PI) {
      delta_angle_A = -2 * PI + delta_angle_A;
    }
    else if (delta_angle_A < -PI) {
      delta_angle_A = 2 * PI - delta_angle_A;
    }
    
    float delta_angle_B = new_angles.y - angle_B;
    if (delta_angle_B > PI) {
      delta_angle_B = -2 * PI + delta_angle_B;
    }
    else if (delta_angle_B < -PI) {
      delta_angle_B = 2 * PI - delta_angle_B;
    }

    pid_controller_angle_A.reference = 0.0;
    pid_controller_angle_A.measurement = -delta_angle_A;
    pid_controller_angle_A.update();
    motor_pwm_A = pid_controller_angle_A.u * 1 * 1023;


    pid_controller_angle_B.reference = 0.0;
    pid_controller_angle_B.measurement = -delta_angle_B;
    pid_controller_angle_B.update();
    motor_pwm_B = pid_controller_angle_B.u * 1 * 1023;
  }

  PVector position_of_effector(float A, float B) {
    PVector p = new PVector(0, 0, 0);
    float x = -cos(A + PI/2) * offset_horizontal + cos(A) * cos(B) * effector_length;
    float y = -sin(A + PI/2) * offset_horizontal + sin(A) * cos(B) * effector_length;
    float z = offset_vertical + sin(B) * effector_length;

    p.set(x, y, z);
    return p;
  }

  PVector robot_angles(PVector target) {
    PVector a = new PVector(0, 0);

    float B = asin((target.z - offset_vertical) / effector_length);
    if (Float.isNaN(B)) {
      B = PI/2;
    }

    float A = atan2(offset_horizontal * target.x + cos(B) * effector_length * target.y, cos(B) * effector_length * target.x - offset_horizontal * target.y);

    if (A < 0) {
      A = 2*PI + A;
    }
    if (B < 0) {
      B = 2*PI + B;
    }

    a.set(A, B);
    return a;
  }

  void update_positions() {
    position = position_of_effector(angle_A, angle_B);
    new_angles = robot_angles(points.points.get(step));
  }

  void control() {
    if (run_start && !run_end) {
      if (control_mode == robot_control_mode.human) {
        human_control();
      } else if (control_mode == robot_control_mode.pwm) {
        pwm_control();
      } else if (control_mode == robot_control_mode.angles) {
        angle_control();
      }
    } else {
      motor_pwm_A = 0;
      motor_pwm_B = 0;
    }
  }

  void update() {
    update_positions();
    control();

    motor_pwm_A = constrain(motor_pwm_A, -1023, 1023);
    motor_pwm_B = constrain(motor_pwm_B, -1023, 1023);

    torque_A = motor_pwm_A / motor_pwm_max * torque_motor_max_A * gearbox_efficiency + motor_K_A * angular_speed_A * gearbox_ratio;
    torque_B = motor_pwm_B / motor_pwm_max * torque_motor_max_B * gearbox_efficiency + motor_K_B * angular_speed_B * gearbox_ratio;

    float angular_speed_A_previous = angular_speed_A;
    angular_speed_A = angular_speed_A + torque_A / rotational_inertia_A * dt;
    float angular_speed_B_previous = angular_speed_B;
    angular_speed_B = angular_speed_B + torque_B / rotational_inertia_B * dt;

    delta_angle_A = (angular_speed_A_previous + angular_speed_A) / 2 * dt;
    angle_A =  (angle_A + delta_angle_A) % (2 * PI);
    if (angle_A < 0) {
      angle_A = 2 * PI + angle_A;
    }

    delta_angle_B = (angular_speed_B_previous + angular_speed_B) / 2 * dt;
    angle_B =  (angle_B + delta_angle_B) % (2 * PI);
    if (angle_B < 0) {
      angle_B = 2 * PI + angle_B;
    }

    time = float(millis())/1000;
    if (run_end) {
      time_run = time - time_start;
    } else if (run_start) {
      time_run = time - time_start;
      time_end = time_run;
      path.add(new PVector(position.x, position.y, position.z));
    } else {
      time_run = 0;
    }

    update_positions();
  }

  void draw() {
    render.pushMatrix();
    robot_draw(render);
    render.popMatrix();
    path_draw(render);
    augmented_display_draw(render);
  }

  void servomotor_draw (PGraphics render) {
    render.pushMatrix();
    box_draw(render, 120, 120, 8, white_colors);
    render.translate(0, 0, 8);
    box_draw(render, 100, 100, 35, gems_colors);
    render.translate(0, 0, 35);
    box_draw(render, 120, 120, 8, white_colors);
    render.translate(0, 0, 8);
    box_draw(render, 60, 90, 30, white_colors);
    render.translate(0, 0, 30);
    cylinder_draw(render, 30, 15, white);
    render.translate(0, 0, 15);
    cylinder_draw(render, 40, 4, white);
    render.popMatrix();
  }

  void robot_draw (PGraphics render) {
    render.pushMatrix();
    servomotor_draw(render);
    render.rotateZ(angle_A);
    render.translate(0, 25 - offset_horizontal, offset_vertical);
    render.rotateX(PI/2);
    render.pushMatrix();
    render.rotateZ(angle_B);
    servomotor_draw(render);
    render.popMatrix();

    render.translate(0, 35, 105);
    render.rotateX(PI/2);
    box_draw(render, 30, 10, offset_vertical - 65, blue_colors);
    render.translate(0, -5, offset_vertical - 70);
    render.rotateX(PI/2);
    box_draw(render, 30, 10, 110 + offset_horizontal, blue_colors);
    render.popMatrix();
  }

  void path_draw(PGraphics render) {
    render.pushMatrix();
    render.strokeWeight(10);
    render.stroke(0, 0, 255);
    for (int i = 0; i < path.size(); i++) {
      render.point(path.get(i).x, path.get(i).y, path.get(i).z);
    }
    render.popMatrix();
  }

  void augmented_display_draw(PGraphics render) {
    render.strokeWeight(20);
    render.stroke(255, 100, 100);
    render.pushMatrix();
    render.rotateZ(angle_A);
    render.translate(0, -offset_horizontal, offset_vertical);
    render.rotateX(PI/2);
    render.rotateZ(angle_B);
    render.rotateY(PI/2);
    render.stroke(255, 100, 100);
    render.line(0, 0, 0, 0, 0, effector_length);
    render.popMatrix();

    render.pushMatrix();
    render.fill(255, 255, 0);
    render.translate(robot.position.x, robot.position.y, robot.position.z);
    render.sphere(5);
    render.popMatrix();
  }
}

void cylinder_draw (PGraphics render, float r, float h, color rgb) {
  render.rectMode(CENTER);

  int N = 30;
  float A = 2 * PI / N;
  float C = 2 * tan(A/2) * r;

  render.pushMatrix();

  render.strokeWeight(stroke_width);
  render.stroke(dark_gray);
  render.fill(rgb);

  render.circle(0, 0, 2*r);
  render.translate(0, 0, h);
  render.circle(0, 0, 2*r);

  render.noStroke();

  render.translate(0, 0, -h/2);
  render.rotateX(PI/2);

  for (int i = 0; i < N; i++) {
    render.rotateY(A);
    render.translate(0, 0, r);
    render.rect(0, 0, C, h);
    render.translate(0, 0, -r);
  }
  render.popMatrix();
}

void box_draw (PGraphics render, float a, float b, float c, color [] RGBs) {
  render.rectMode(CENTER);

  render.pushMatrix();

  render.noStroke();
  render.fill(RGBs[0]);
  render.rect(0, 0, a, b);
  render.translate(0, 0, c);
  render.fill(RGBs[1]);
  render.rect(0, 0, a, b);

  render.translate(0, 0, -c/2);
  render.rotateX(PI/2);

  render.strokeWeight(stroke_width);
  render.stroke(dark_gray);

  render.translate(0, 0, b/2);
  render.fill(RGBs[2]);
  render.rect(0, 0, a, c);
  render.translate(0, 0, -b);
  render.fill(RGBs[4]);
  render.rect(0, 0, a, c);
  render.translate(0, 0, b/2);

  render.rotateY(PI/2);
  render.translate(0, 0, a/2);
  render.fill(RGBs[3]);
  render.rect(0, 0, b, c);
  render.translate(0, 0, -a);
  render.fill(RGBs[5]);
  render.rect(0, 0, b, c);
  render.translate(0, 0, -a/2);

  render.popMatrix();
}

void grid(PGraphics render, float a, float b, float c, color rgb) {
  render.pushMatrix();
  render.rectMode(CENTER);
  render.noStroke();
  render.fill(gray);
  render.rect(0, 0, a, b);
  render.translate(0, 0, 0);
  render.strokeWeight(10);
  render.stroke(rgb);
  for (int i = 0; i < floor(a/c)+1; i++) {
    render.line(-a/2 + i*c, -b/2, -a/2 + i*c, b/2);
  }
  for (int i = 0; i < floor(b/c)+1; i++) {
    render.line(-a/2, -b/2 + i*c, a/2, -b/2 + i*c);
  }
  render.popMatrix();
}

class Points {
  Robot myRobot;

  ArrayList<PVector> points = new ArrayList<PVector>();
  ArrayList<Boolean> points_state = new ArrayList<Boolean>();

  int N_points = 12;
  float delta_angle = 2 * PI / N_points;
  float circle_R = 200;

  float center_A = PI/2;
  float center_B = PI/8;

  float point_radius = 10;

  Points(Robot myRobot) {
    this.myRobot = myRobot;

    PMatrix3D rotation_Z = new PMatrix3D(
      cos(center_A), -sin(center_A), 0, 0,
      sin(center_A), cos(center_A), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
      );

    PMatrix3D rotation_Y = new PMatrix3D(
      cos(-center_B), 0, sin(-center_B), 0,
      0, 1, 0, 0,
      -sin(-center_B), 0, cos(-center_B), 0,
      0, 0, 0, 1
      );

    float F = -asin(myRobot.offset_horizontal/sqrt(pow(myRobot.offset_horizontal, 2) + pow(myRobot.effector_length, 2) - pow(circle_R, 2)));
    PMatrix3D rotation_X = new PMatrix3D(
      cos(F), -sin(F), 0, 0,
      sin(F), cos(F), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
      );

    PMatrix3D rotation_YZX = rotation_Z.get();
    rotation_YZX.apply(rotation_Y);
    rotation_YZX.apply(rotation_X);

    for (int i = 0; i < N_points; i++) {
      PVector C = new PVector( 0, circle_R * sin(delta_angle * i), circle_R * cos(delta_angle * i));
      PVector P = new PVector(0, 0, 0);
      rotation_YZX.mult(C, P);
      points.add(P.add(position_of_center(center_A, center_B)));
      points_state.add(false);
    }
  }

  PVector position_of_center(float A, float B) {
    PVector p = new PVector(0, 0, 0);
    float x = -cos(A + PI/2) * myRobot.offset_horizontal + cos(A) * cos(B) * sqrt(pow(myRobot.effector_length, 2) - pow(circle_R, 2));
    float y = -sin(A + PI/2) * myRobot.offset_horizontal + sin(A) * cos(B) * sqrt(pow(myRobot.effector_length, 2) - pow(circle_R, 2));
    float z = myRobot.offset_vertical + sin(B) * sqrt(pow(myRobot.effector_length, 2) - pow(circle_R, 2));

    p.set(x, y, z);
    return p;
  }

  void draw() {
    render.pushMatrix();
    for (int i = 0; i < points.size(); i++) {
      PVector point = points.get(i);
      Boolean point_state = points_state.get(i);
      render.pushMatrix();
      render.translate(point.x, point.y, point.z);
      if (i == step && !point_state) {
        sphere_draw(point_radius, color(250, 250, 50));
      } else if (point_state) {
        sphere_draw(point_radius, color(50, 200, 50));
      } else {
        sphere_draw(point_radius, color(235, 50, 50));
      }
      render.popMatrix();
    }
    render.popMatrix();

    show_points_shell();
  }

  void show_points_shell() {
    render.pushMatrix();
    render.strokeWeight(10);
    render.stroke(0, 0, 0);
    int n = 50;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        PVector W = myRobot.position_of_effector(i*2*PI/n, j*2*PI/n);
        render.point(W.x, W.y, W.z);
      }
    }
    render.popMatrix();
  }

  void check() {
    PVector point = points.get(step);
    float err = sqrt(pow(point.x - myRobot.position.x, 2) + pow(point.y  - myRobot.position.y, 2) + pow(point.z  - myRobot.position.z, 2));

    if (err < point_radius) {
      points_state.set(step, true);
      if (step < points.size()-1) {
        step = step + 1;
      } else {
        myRobot.run_end = true;
      }
    }
  }
}

class PID_Controller {
  float reference = 0.0;
  float measurement = 0.0;
  float error = 0.0;
  float error_previous = 0.0;
  float error_difference = 0.0;

  float K_p = 0.0;
  float K_i = 0.0;
  float K_d = 0.0;

  float u_p = 0.0;
  float u_i = 0.0;
  float u_d = 0.0;

  float u_i_min = -1.0;
  float u_i_max = 1.0;

  float u = 0.0;

  PID_Controller(float K_p, float K_i, float K_d) {
    this.K_p = K_p;
    this.K_i = K_i;
    this.K_d = K_d;
  }

  void update() {
    error = reference - measurement;
    error_difference = error - error_previous;
    error_previous = error;

    u_p = K_p * error;
    u_i = constrain(u_i + K_i * error * dt, u_i_min, u_i_max);
    u_d = K_d * error_difference / dt;

    u = constrain(u_p + u_i + u_d, -1, 1);
  }
}
