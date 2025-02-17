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
    speed,
    orientation_1,
    orientation_2,
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

  camera = new PeasyCam(this, 0, 0, 0, 3000);

  camera.rotateZ(PI/2);
  camera.rotateY(0);
  camera.rotateX(-PI/3);

  mat_A = getMatrix();

  frameRate(fps);
  windowTitle("GEMS mobile robot");
  windowResizable(false);

  robot = new Robot(render, new PVector(0, 0, 0), 12*PI/12);
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
    render.ambientLight(170, 170, 170);
    render.perspective(fov, aspect, near, far);

    //CS();

    grid(render, 5000, 5000, 250, gray);
    render.translate(-700, -400, 86);

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
  text(String.format("Speed: %.3f m/s", robot.speed_linear), 60, 60);

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

  final float mass = 1.167;
  final float wheel_radius = 0.085;
  final float wheel_base = 0.2;
  final float motor_pwm_max = 1023.0;

  float gearbox_ratio = 243.0;
  float gearbox_efficiency = 0.75;

  float torque_motor_max_L = 0.015;
  float torque_motor_max_R = 0.015;

  float torque_wheel_L = 0.0;
  float torque_wheel_R = 0.0;

  float motor_K_L = -0.015/120;
  float motor_K_R = -0.015/120;

  float motor_pwm_L = 0.0;
  float motor_pwm_R = 0.0;

  float force_L = 0;
  float force_R = 0;

  float speed_L = 0;
  float speed_R = 0;

  float speed_linear = 0;
  float speed_rotational = 0;

  float rotate = 0.0;
  float move = 0.0;

  PVector position;
  float orientation;

  Boolean run_start = false;
  Boolean run_end = false;
  float time_start = 0;
  float time_end = 0;
  float time_run = 0;
  float time = 0;

  ArrayList<PVector> path = new ArrayList<PVector>();

  robot_control_mode control_mode = robot_control_mode.orientation_2;

  PID_Controller pid_controller_speed_L = new PID_Controller(500.0, 100.0, 0.0);
  PID_Controller pid_controller_speed_R = new PID_Controller(500.0, 100.0, 0.0);

  PID_Controller pid_controller_orientation = new PID_Controller(3, 0.0, 1.5);
  PVector position_relative = new PVector(0, 0, 0);
  float orientation_relative = 0.0;

  PVector position_relative_future = new PVector(0, 0, 0);
  float orientation_relative_future = 0.0;
  float action_mag = 0;


  Robot(PGraphics render, PVector position, float orientation) {
    this.render = render;
    this.position = position;
    this.orientation = orientation;
  }

  void human_control() {
    if (keys['q']) {
      motor_pwm_L = 1023;
    } else if (keys['a']) {
      motor_pwm_L = -1023;
    } else {
      motor_pwm_L = 0;
    }

    if (keys['e']) {
      motor_pwm_R = 1023;
    } else if (keys['d']) {
      motor_pwm_R = -1023;
    } else {
      motor_pwm_R = 0;
    }
  }

  void pwm_control() {
    if (time_run < 7.55) {
      motor_pwm_L = 1023;
      motor_pwm_R = 1023;
    } else if (time_run < 8) {
      motor_pwm_L = -1023;
      motor_pwm_R = -1023;
    } else if (time_run < 9.08) {
      motor_pwm_L = 1023;
      motor_pwm_R = -1023;
    } else if (time_run < 9.475) {
      motor_pwm_L = -1023;
      motor_pwm_R = 1023;
    } else if (time_run < 11) {
      motor_pwm_L = 1023;
      motor_pwm_R = 1023;
    } else if (time_run < 23.3) {
      motor_pwm_L = 1023;
      motor_pwm_R = 798;
    } else if (time_run < 25.5) {
      motor_pwm_L = 1023;
      motor_pwm_R = 1023;
    } else {
      motor_pwm_L = 0;
      motor_pwm_R = 0;
    }
  }

  void speed_control() {
    if (time_run < 7.7) {
      pid_controller_speed_L.reference = 0.195;
      pid_controller_speed_R.reference = 0.195;
    } else if (time_run < 8.2) {
      pid_controller_speed_L.reference = 0;
      pid_controller_speed_R.reference = 0;
    } else if (time_run < 9.35) {
      pid_controller_speed_L.reference = 0.195;
      pid_controller_speed_R.reference = -0.195;
    } else if (time_run < 10) {
      pid_controller_speed_L.reference = 0.0;
      pid_controller_speed_R.reference = 0.0;
    } else if (time_run < 12) {
      pid_controller_speed_L.reference = 0.195;
      pid_controller_speed_R.reference = 0.195;
    } else if (time_run < 24.0) {
      pid_controller_speed_L.reference = 0.197;
      pid_controller_speed_R.reference = 0.155;
    } else if (time_run < 26.5) {
      pid_controller_speed_L.reference = 0.197;
      pid_controller_speed_R.reference = 0.197;
    } else {
      pid_controller_speed_L.reference = 0.0;
      pid_controller_speed_R.reference = 0.0;
    }

    pid_controller_speed_L.measurement = speed_L;
    pid_controller_speed_R.measurement = speed_R;

    pid_controller_speed_L.update();
    pid_controller_speed_R.update();

    motor_pwm_L = pid_controller_speed_L.u * 1023;
    motor_pwm_R = pid_controller_speed_R.u * 1023;
  }

  void orientation_1_control() {
    pid_controller_orientation.reference = 0.0;

    position_relative = PVector.sub(points.points.get(step), position);
    orientation_relative = relative_orientation_of_vector(position_relative, orientation);

    pid_controller_orientation.measurement = orientation_relative;

    pid_controller_orientation.update();

    motor_pwm_L = 1023 - pid_controller_orientation.u * 1023 * 5 ;
    motor_pwm_R = 1023 + pid_controller_orientation.u * 1023 * 5;
  }

  void orientation_2_control() {
    pid_controller_orientation.reference = 0.0;

    position_relative = PVector.sub(points.points.get(step), position);
    orientation_relative = relative_orientation_of_vector(position_relative, orientation);

    if (step + 1 < points.points.size()) {
      action_mag = orientation_relative_future * 1 * 1023 ;
    } else {
      action_mag = 0;
    }

    pid_controller_orientation.measurement = orientation_relative;

    pid_controller_orientation.update();

    motor_pwm_L = 1023 - pid_controller_orientation.u * 1023 * 5 - action_mag;
    motor_pwm_R = 1023 + pid_controller_orientation.u * 1023 * 5 + action_mag;
  }

  float relative_orientation_of_vector(PVector p, float base) {
    float result = 0.0;
    if (p.x == 0 && p.y == 0) {
      result = 0.0;
    } else if (p.x == 0 && p.y < 0) {
      result = PI/2;
    } else if (p.x < 0 && p.y == 0) {
      result = PI;
    } else if (p.x == 0 && p.y > 0) {
      result = 3 / 2 * PI;
    } else if (p.x < 0 && p.y == 0) {
      result = 0.0;
    } else if (p.x > 0 && p.y < 0) {
      result = atan(-p.y/p.x);
    } else if (p.x < 0 && p.y < 0) {
      result = PI - atan(p.y/p.x);
    } else if (p.x < 0 && p.y > 0) {
      result = PI + atan(p.y/-p.x);
    } else if (p.x > 0 && p.y > 0) {
      result = 2 * PI - atan(p.y/p.x);
    }
    result =  (result + base)%(2*PI);
    if (result > PI) {
      result = result - (2 * PI);
    }
    result = -result;

    return result;
  }

  void update_positions() {
    position_relative = PVector.sub(points.points.get(step), position);
    orientation_relative = relative_orientation_of_vector(position_relative, orientation);

    if (step + 1 < points.points.size()) {
      position_relative_future = PVector.sub(points.points.get(step + 1), position);
      orientation_relative_future = relative_orientation_of_vector(position_relative_future, orientation);
    } else {
      position_relative_future.set(0, 0, 0);
      orientation_relative_future = 0.0;
    }
  }

  void control() {
    if (run_start && !run_end) {
      if (control_mode == robot_control_mode.human) {
        human_control();
      } else if (control_mode == robot_control_mode.pwm) {
        pwm_control();
      } else if (control_mode == robot_control_mode.speed) {
        speed_control();
      } else if (control_mode == robot_control_mode.orientation_1) {
        orientation_1_control();
      } else if (control_mode == robot_control_mode.orientation_2) {
        orientation_2_control();
      }
    } else {
      motor_pwm_L = 0;
      motor_pwm_R = 0;
    }
  }

  void update() {
    update_positions();
    control();

    motor_pwm_L = constrain(motor_pwm_L, -1023, 1023);
    motor_pwm_R = constrain(motor_pwm_R, -1023, 1023);

    torque_wheel_L = motor_pwm_L / motor_pwm_max * torque_motor_max_L * gearbox_efficiency + motor_K_L * speed_L / (2 * PI * wheel_radius) * gearbox_ratio;
    force_L = torque_wheel_L / wheel_radius;

    torque_wheel_R = motor_pwm_R / motor_pwm_max * torque_motor_max_R * gearbox_efficiency + motor_K_R * speed_R / (2 * PI * wheel_radius) * gearbox_ratio;
    force_R = torque_wheel_R / wheel_radius;

    speed_L = speed_L + force_L * 2 / mass * dt;
    speed_R = speed_R + force_R * 2 / mass * dt;

    float speed_linear_previous = speed_linear;
    speed_linear = (speed_L + speed_R) / 2;

    float speed_rotational_previous = speed_rotational;
    speed_rotational = (speed_L - speed_R) / (2 * wheel_radius);

    rotate = (speed_rotational_previous + speed_rotational) / 2 * dt;

    orientation =  (orientation + rotate) % (2 * PI);
    if (orientation < 0) {
      orientation = 2 * PI + orientation;
    }

    move = 1000 * (speed_linear_previous + speed_linear) / 2 * dt;
    position.set(position.x + move * cos(orientation), position.y + move * sin(orientation), position.z);

    time = float(millis())/1000;
    if (run_end) {
      time_run = time - time_start;
    } else if (run_start) {
      time_run = time - time_start;
      time_end = time_run;
      path.add(new PVector(position.x, position.y));
    } else {
      time_run = 0;
    }

    update_positions();
  }

  void draw() {
    render.pushMatrix();
    render.translate(position.x, position.y, 0);
    render.rotateZ(orientation);
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
    box_draw(render, 60, 90, 25, white_colors);
    render.translate(0, 0, 25);
    cylinder_draw(render, 85, 35, blue);
    render.popMatrix();
  }

  void robot_draw (PGraphics render) {
    render.pushMatrix();
    render.rotateX(-PI/2);
    servomotor_draw(render);
    render.rotateX(PI);
    render.rotateZ(PI);
    servomotor_draw(render);
    render.translate(105, 70, -7.5);
    cylinder_draw(render, 15, 15, blue);
    render.translate(0, 0, 7.5);
    render.rotateZ(PI/8);
    render.translate(-25, 0, -4);
    box_draw(render, 60, 10, 8, blue_colors);
    render.popMatrix();
  }

  void path_draw(PGraphics render) {
    render.pushMatrix();
    render.strokeWeight(10);
    render.stroke(0, 0, 255);
    for (int i = 0; i < path.size(); i++) {
      render.point(path.get(i).x, path.get(i).y, 0);
    }
    render.popMatrix();
  }

  void augmented_display_draw(PGraphics render) {
    render.strokeWeight(20);
    render.stroke(50, 50, 255);
    render.pushMatrix();
    render.translate(position.x, position.y, 0);
    render.rotateZ(orientation);
    render.line(100, 0, 0, 150, 0, 0);
    render.popMatrix();

    render.strokeWeight(20);
    render.stroke(50, 200, 50);
    render.pushMatrix();
    render.translate(position.x, position.y, 0);
    render.rotateZ(orientation_relative + orientation);
    render.line(150, 0, 0, 200, 0, 0);
    render.popMatrix();

    render.noFill();
    render.strokeWeight(20);
    render.stroke(50, 200, 50);
    render.pushMatrix();
    render.translate(position.x + position_relative.x, position.y + position_relative.y, 0);
    render.circle(0, 0, 64);
    render.popMatrix();

    render.strokeWeight(20);
    render.stroke(255, 250, 50);
    render.pushMatrix();
    render.translate(position.x, position.y, 0);
    render.rotateZ(orientation_relative_future + orientation);
    render.line(200, 0, 0, 250, 0, 0);
    render.popMatrix();

    render.noFill();
    render.strokeWeight(20);
    render.stroke(255, 250, 50);
    render.pushMatrix();
    render.translate(position.x + position_relative_future.x, position.y + position_relative_future.y, 0);
    render.circle(0, 0, 64);
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
  ArrayList<PVector> points = new ArrayList<PVector>();
  ArrayList<Boolean> points_state = new ArrayList<Boolean>();

  float point_radius = 30;
  float wheel_base = 200;
  
  Robot myRobot;

  Points(Robot myRobot) {
    this.myRobot = myRobot;
    
    float line = 1400.0;
    float radius = line/2;
    int N_line_points = 5;
    float delta_line = line/N_line_points;
    float arc = PI;
    int N_arc_points = 7;
    float delta_arc = arc/N_arc_points;

    int N_all_points = N_line_points + N_arc_points + 2;

    for (int i = 0; i < N_all_points; i++) {
      points_state.add(false);
    }

    for (int i = 1; i < N_line_points + 1; i++) {
      points.add(new PVector(i * delta_line, 0, 0));
    }

    for (int i = 0; i < N_arc_points + 1; i++) {
      points.add(new PVector(radius + cos(i * delta_arc) * radius, delta_line + sin(i * delta_arc) * radius, 0));
    }

    points.add(new PVector(0, 0, 0));
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
  }

  void check() {
    PVector point = points.get(step);
    float err = sqrt(pow(point.x - myRobot.position.x, 2) + pow(point.y  - myRobot.position.y, 2));
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
