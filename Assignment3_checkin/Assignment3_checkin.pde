int character_x = 0;
int character_y = 0;
int grid_size = 30;
PVector start;
PVector goal;
PVector obstacle_center;
int sample_number = 50;
int obstacle_radius = 2 * grid_size;
int character_radius = 15;
PVector[] sample_points;

float straightDistance(PVector vector_1, PVector vector_2) {
  
  float x_distance = (vector_1.x - vector_2.x) * (vector_1.x - vector_2.x);
  float y_distance = (vector_1.y - vector_2.y) * (vector_1.y - vector_2.y);
  float distance = sqrt(x_distance + y_distance);
  return distance;
  
}

// Check if the sample point has collision with obstacle
boolean isFeasible(PVector sample_point) {
  boolean result = false;
  float distance = straightDistance(obstacle_center, sample_point);
  print(distance + "\n");
  if (distance > (obstacle_radius + character_radius)) {
    result = true;
  }
  
  return result;
}


void setup() {
  
  size(700,700);
  character_x = 50 + grid_size;
  character_y = 50 + 19 * grid_size;
  start = new PVector(character_x, character_y);
  goal = new PVector(600,100);
  obstacle_center = new PVector(width/2, height/2);
  sample_points = new PVector[sample_number];
  PVector sample_point;
  for (int i = 0; i < sample_number; i++) {
    sample_point = new PVector(random(55,595), random(55,595));
    
    while (isFeasible(sample_point) == false) {
      sample_point = new PVector(random(50,600), random(50,600));
    }
    
    sample_points[i] = sample_point;
    
  }
  
}

void draw() {
  
  fill(255,255,255);
  rect(50,50,600,600);
  
  fill(0,0,255);
  circle(width/2, height/2, obstacle_radius); // obstacle
  
  fill(255,0,0);
  circle(600, 100, 15); // goal
  
  fill(0,255,0);
  circle(character_x, character_y, 0.5 * grid_size);
  
  for (int i = 0; i < sample_number; i++) {
    fill(0,0,0);
    circle(sample_points[i].x, sample_points[i].y, 10);
  }
 
}
