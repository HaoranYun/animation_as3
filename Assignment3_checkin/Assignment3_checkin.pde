import java.util.*; 

int character_x = 0;
int character_y = 0;
int grid_size = 30;
PVector start;
PVector goal;
Milestone GOAL;
Milestone START;
PVector obstacle_center;
int sample_number = 50;
int obstacle_radius = 2 * grid_size;
int character_radius = 15;
Milestone[] sample_points;
int[][] paths_status = new int[sample_number+2][sample_number+2];

PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new MyCompare());
ArrayList<Milestone> path = new ArrayList<Milestone>();

float vel = 5;
Agent agent;
boolean moving = false;
int idx = 0;


// return distance between two points. 

float straightDistance(PVector vector_1, PVector vector_2) {
  
  float x_distance = (vector_1.x - vector_2.x) * (vector_1.x - vector_2.x);
  float y_distance = (vector_1.y - vector_2.y) * (vector_1.y - vector_2.y);
  float distance = sqrt(x_distance + y_distance);
  return distance;
  
}

// Check if the sample point has collision with obstacle
// The checking is based on using character as an point and all the other positions
// rather than with its radius. 
// return false if collision occurs; return true otherwise
boolean isFeasible(PVector sample_point) {
  boolean result = false;
  float distance = straightDistance(obstacle_center, sample_point);
  //print(distance + "\n");
  if (distance > (obstacle_radius + character_radius)) { // by using obstacle_radius + character_radius, we could take character as a point. 
    result = true;
  }
  
  return result;
}


// Check if the path between two feasible sample point would has
// collision with obstacle.
// return false if collisions occurs; return true otherwise. 
boolean checkPath(PVector point_1, PVector point_2) {
  boolean result = true;

  PVector check_point = new PVector(point_1.x, point_1.y);
  float x_distance = point_2.x - point_1.x;
  float y_distance = point_2.y - point_1.y;
  
  float step_num = max(abs(x_distance), abs(y_distance));
  float x_step_size = x_distance / step_num;
  float y_step_size = y_distance / step_num;
  for (int i = 1; i < step_num; i++) {
    check_point.x += x_step_size;
    check_point.y += y_step_size;
    //print("check point_x: " + check_point.x + ", check point_y: " + check_point.y + "\n");
    if (isFeasible(check_point) == false) {
      result = false;
      break;
    }
    
  }
  
  
  return result;
}


boolean A_star(){
  
  // add start point to the queue
  START.isVisited = true;
  fringe.add(START);
  println("a*!!");
  int iter = 0;
  
  while (fringe.size() >0){
    iter++;
    println(iter);
    
    Milestone curr = fringe.poll();
    if(curr.isGoal) {
      println("FINISH!!");
      return true;
    }
    int num_neighbors = curr.neighbors.size();
    for(int i = 0; i < num_neighbors; i++){
      Milestone child = curr.neighbors.get(i);
      if(!child.isVisited){
        child.parent = curr;
        child.isVisited = true;
        fringe.add(child);
      }
    }
  }
  
  println("A* FINISH!!");
  return true;
}

void setup() {
  
  size(700,700);
  character_x = 50 + grid_size;
  character_y = 50 + 19 * grid_size;
  start = new PVector(character_x, character_y);
  goal = new PVector(600,100);
  obstacle_center = new PVector(width/2, height/2);
  
  sample_points = new Milestone[sample_number+2];
  
  GOAL = new  Milestone(goal);
  START = new  Milestone(start);
  GOAL.isGoal = true;
  
  sample_points[0] = START;
  sample_points[1] = GOAL;

  PVector sample_point;
  for (int i = 0; i < sample_number+2; i++) {
    for (int j = 0; j < sample_number+2; j++) {
       paths_status[i][j] = 0;
    }
  }
  
  // generate random sample points
  for (int i = 2; i < sample_number+2; i++) {
    sample_point = new PVector(random(55,595), random(55,595));
    
    while (isFeasible(sample_point) == false) {
      sample_point = new PVector(random(55,595), random(55,595));
    }
    
    sample_points[i] = new  Milestone(sample_point);
    
    
  }
  
  // find all the paths that do not has collision with obstacles
  for (int i = 0; i < sample_number + 2; i++) {
    for (int j = 0; j < sample_number + 2; j++) {
      
      if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10) {
         if (checkPath(sample_points[i].pos, sample_points[j].pos) == true) {
           
           sample_points[i].neighbors.add(sample_points[j]);
           paths_status[i][j] = 1;
        
        }
      }
      //print(paths_status[i][j] + "\n");
      
    }
  }
  
  
  
  
    //print(checkPath(start, goal));
    
  
  A_star();
  
  Milestone curr = GOAL;
  while(curr.parent!=null){
     path.add(curr);
     curr = curr.parent;
  }
  path.add(curr);
  Collections.reverse(path);
  
  println("len " + path.size());
  
  agent = new Agent(start);
  agent.setVel(PVector.sub(path.get(idx+1).pos,path.get(idx).pos).normalize());
  idx ++;
  
}

void draw() {
  
  fill(255,255,255);
  rect(50,50,600,600); // rectangle
  
  fill(0,0,255);
  circle(width/2, height/2, obstacle_radius); // obstacle

  
  for (int i = 2; i < sample_number+2; i++) {
    fill(0,0,0);
    circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
  }
  
  //for (int i = 0; i < sample_number + 2; i ++) {
  //  for (int j = 0; j < sample_number + 2; j ++) {
      
  //    if ( paths_status[i][j] == 1 ) {
  //      stroke(218,122,214);
  //      line(sample_points[i].pos.x, sample_points[i].pos.y, sample_points[j].pos.x, sample_points[j].pos.y);
  //    }
  //   }
  //}
  
  
  fill(200,200,20);
  stroke(200,200,20);
  
  Milestone curr = GOAL;
  Milestone prev = GOAL;
  
  while(curr.parent!=null){
     circle(curr.pos.x, curr.pos.y, 15);
     prev = curr;
     curr = curr.parent;
     line(prev.pos.x, prev.pos.y, curr.pos.x,curr.pos.y);
  }
  
  fill(255,150,50);
  if(moving){
    agent.update();
    circle(agent.pos.x, agent.pos.y, 15);
  }
  
  
  //print(checkPath(start, goal));
 
  fill(255,0,0);
  circle(600, 100, 15); // goal
  
  fill(0,255,0);
  circle(character_x, character_y, 0.5 * grid_size); // start
  
  noStroke();
  
  
}

void keyPressed(){
  if(keyCode == ENTER){
    moving = !moving;
  }
}
