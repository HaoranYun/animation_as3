import java.util.*; 

int character_x = 0;
int character_y = 0;
int grid_size = 30;

PVector obstacle_center;
int sample_number = 30;
float obstacle_radius = 50;
float character_radius = 7.5;
Milestone[] sample_points;
int[][] paths_status = new int[sample_number+2][sample_number+2];

PVector movingGoal1 = new PVector(600,100);
PVector movingGoal2 = new PVector(100,300);


float vel = 5;
ArrayList<Agent> agents = new ArrayList<Agent>();
boolean moving = false;
ArrayList<Obstacle> obstacles =new  ArrayList<Obstacle>();

int searchLimit = 1000;
//ArrayList<Agent2> agents2 = new ArrayList<Agent2>();
// return distance between two points. 

float straightDistance(PVector vector_1, PVector vector_2) {
  
  float x_distance = (vector_1.x - vector_2.x) * (vector_1.x - vector_2.x);
  float y_distance = (vector_1.y - vector_2.y) * (vector_1.y - vector_2.y);
  float distance = sqrt(x_distance + y_distance);
  return distance;
  
}

// Check if the sample point has collision with obstacle
// The checking is based on using character as an point and all the other positions
// rather than with its . 
// return false if collision occurs; return true otherwise
boolean isFeasible(PVector sample_point) {
  boolean result = true;
  float distance;
  //print(distance + "\n");
  float r;
  for(int i = 0; i< obstacles.size(); i ++){
     distance = straightDistance( obstacles.get(i).center, sample_point);
    r= obstacles.get(i).r + character_radius ;
    if(distance < (57.5)) result = false;
    
  }
  return result;
}

boolean checkPath_CCD(PVector p1, PVector p2){
  
  float a,b,c;
  float i,mu1, mu2;
  PVector dp,center;
  
  float r;
 
  boolean feasible = true;
        
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    r =57.5;
    
    
    dp = PVector.sub(p2,p1);
    a = dp.x*dp.x + dp.y * dp.y;
    b = 2*(dp.x*(p1.x - center.x) +dp.y*(p1.y - center.y));
    c = PVector.dot(center,center);
    c += PVector.dot(p1,p1);
    c -= 2* PVector.dot(center,p1);
    c -= r*r;
    i = b*b - 4*a*c;
    if(i>=0)
    //println("i:" +i);
  
    mu1 =0;
    mu2 = 0;
  
    if(i == 0){
      mu1 = -b/2/a;
      if(mu1> 0 && mu1 <1) return false;
    
    }
  
    else{
      mu1 = (-b + sqrt(i))/2/a;
      mu2 = (-b - sqrt(i))/2/a;
      if((mu1 >0 && mu1<1) || (mu2>0&&mu2<1)) return false;

    
    }
  //if(result == false) println(p1 + " " + p2 + "mu " +mu1 +" " + mu2 );
  }
    return true;


}


// Check if the path between two feasible sample point would has
// collision with obstacle.
// return false if collisions occurs; return true otherwise. 








void setup() {
  
  size(700,700);
  character_x = 50 + grid_size;
  character_y = 50 + 19 * grid_size;
  obstacle_center = new PVector(width/2, height/2);
  
  
  for(int i = 0; i< 4; i++){
    obstacles.add(new Obstacle(obstacle_radius, 400, 100 + 100 *i));
  }
  
 for(int i = 0; i< 1; i++){
    obstacles.add(new Obstacle(obstacle_radius, 520, 400));
  }
   //obstacles.add( new Obstacle(100, 300, 350));
   //obstacles.add(new Obstacle(100, 200, 600));
   //obstacles.add( new Obstacle(100, 200, 200));
  
  
  sample_points = new Milestone[sample_number];
  float start_x = 80;
  float start_y = 500;
  
  for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(80 + 20*(i%5), 300 + 20* ((int)i/5 )),character_radius));
    //agents2.add(new Agent2(new PVector(80 + 15*(i%5), 300 + 15* ((int)i/5 ))));
    //agents2.get(i).runBFS = true;
  }
  
 for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(400 + 20*((int)i/5), 500 + 20* ((int)i%5 )), character_radius));
    //agents2.add(new Agent2(new PVector(80 + 15*(i%5), 300 + 15* ((int)i/5 ))));
    //agents2.get(i).runBFS = true;
  }

  PVector sample_point;
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
       paths_status[i][j] = 0;
    }
  }
  
  // generate random sample points
  for (int i = 0; i < sample_number; i++) {
    sample_point = new PVector(random(55,595), random(55,595));
    
    while (isFeasible(sample_point) == false) {
      sample_point = new PVector(random(55,595), random(55,595));
    }
    
    sample_points[i] = new  Milestone(sample_point);
  }
  
  // find all the paths that do not has collision with obstacles
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
      
      if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10) {
        
        boolean feasible =  checkPath_CCD(sample_points[i].pos, sample_points[j].pos);
        
         if (feasible) {
           
           sample_points[i].neighbors.add(sample_points[j]);
           paths_status[i][j] = 1;
        
        }
      }

      
    }
  }
  
    int allNoPath = 0;
  for(int i = 0; i < agents.size(); i++){
    //agents.get(i).setGoalForce(new PVector(0,0));
    if(!agents.get(i).reachGoal)
    { 
      //println("research i: " + i);
      agents.get(i).searchPath(agents.get(i).pos,movingGoal1);
      if(agents.get(i).myPath.size()<1) allNoPath ++;
    }
  }
  
  for(int i = 19; i < agents.size(); i++){
    //agents.get(i).setGoalForce(new PVector(0,0));
    if(!agents.get(i).reachGoal)
    { 
      //println("research i: " + i);
      agents.get(i).searchPath(agents.get(i).pos,movingGoal2);
      if(agents.get(i).myPath.size()<1) allNoPath ++;
    }
  
  }
  if(allNoPath > 15) resampleAll();
}

void draw() {
  
  fill(255,255,255);
  rect(50,50,600,600); // rectangle
  
  fill(0,0,255);
  for(int i = 0; i< obstacles.size(); i ++){
    fill(0,0,255);
    circle(obstacles.get(i).center.x,obstacles.get(i).center.y, obstacles.get(i).r * 2);
    
  }


  
  for (int i = 0; i < sample_number; i++) {
    fill(0,0,0);
    circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
    //text((int)sample_points[i].g+", "+(int)sample_points[i].h,sample_points[i].pos.x, sample_points[i].pos.y);
    
  }
  
  
  fill(200,200,20);
  stroke(200,200,20);
  

  
  
  
  
  
  fill(255,150,50);


  for(int i = 0; i < agents.size(); i ++){
      if(moving) agents.get(i).getForce();
      
  }
  for(int i = 0; i < agents.size(); i ++){
      if(moving)
      agents.get(i).update();
      
      stroke(255,0,0);
      //line(agents.get(i).pos.x, agents.get(i).pos.y,agents.get(i).pos.x+agents.get(i).vel.x*10,agents.get(i).pos.y+agents.get(i).vel.y*10);
      circle(agents.get(i).pos.x, agents.get(i).pos.y, 15);
      //stroke(0,255,0);
      if(agents.get(i).myPath.size()>0){
      for(int j = 1; j < agents.get(i).myPath.size(); j++){
        //circle(agents.get(i).myPath.get(j).pos.x, agents.get(i).myPath.get(j).pos.y, 10);
        //line(agents.get(i).myPath.get(j-1).pos.x,agents.get(i).myPath.get(j-1).pos.y,agents.get(i).myPath.get(j).pos.x,agents.get(i).myPath.get(j).pos.y);
      }
      }
  }
  
  for(int i = 0; i < agents.size(); i ++){
    //agents.get(i).collisionSolver();
  }

  
 
  fill(255,0,0);
  circle(agents.get(0).myGoal.pos.x, agents.get(0).myGoal.pos.y,30);
  
  
  noStroke();
  
  fill(255,0,0);
  text("Please press ENTER to check agent's movement.",50,30);
  
}

void keyPressed(){
  if(keyCode == ENTER){
    moving = !moving;
  }
  else if (keyCode == UP){
    for(int i = 0; i < agents.size(); i ++){
      agents.get(i).setMyGoal(PVector.add(new PVector(0,-15),agents.get(i).myGoal.pos));
    }
  }
  else if (keyCode == DOWN){
    for(int i = 0; i < agents.size(); i ++){
      agents.get(i).setMyGoal(PVector.add(new PVector(0,15),agents.get(i).myGoal.pos));
    }
  }
  else if (keyCode == LEFT){
    for(int i = 0; i < agents.size(); i ++){
      agents.get(i).setMyGoal(PVector.add(new PVector(-15,0),agents.get(i).myGoal.pos));
    }
  }
  else if (keyCode == RIGHT){
    for(int i = 0; i < agents.size(); i ++){
      agents.get(i).setMyGoal(PVector.add(new PVector(15,0),agents.get(i).myGoal.pos));
    }
  }
  
}

void mousePressed(){
  obstacles.add(new Obstacle(100, mouseX, mouseY));
  //for(int j = 0; j < sample_number; j ++){
  //   float dist = PVector.sub()sample
  //}
  
    for (int i = 0; i < sample_number; i++) {
      if(!isFeasible(sample_points[i].pos)){
        PVector sample_point = new PVector(random(55,595), random(55,595));
        while (isFeasible(sample_point) == false) {
          sample_point = new PVector(random(55,595), random(55,595));
        }
        sample_points[i] = new Milestone(sample_point);
      }
      
      sample_points[i].neighbors = new ArrayList<Milestone>();
      
    for (int j = 0; j < sample_number; j++) {
      
      if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10) {
        
        boolean feasible =  checkPath_CCD(sample_points[i].pos, sample_points[j].pos);
        
         if (feasible) {
           
           sample_points[i].neighbors.add(sample_points[j]);
           paths_status[i][j] = 1;
        
        }
      }

      
    }
  }
  
  searchLimit = 1000;
  reSearch();
  

}

void resampleAll(){
    println(searchLimit);
    searchLimit--;
    if(searchLimit < 0) return;
    for (int i = 0; i < sample_number; i++) {
    PVector sample_point = new PVector(random(55,595), random(55,595));
    
    while (isFeasible(sample_point) == false) {
      sample_point = new PVector(random(55,595), random(55,595));
    }
    
    sample_points[i] = new  Milestone(sample_point);
  }
  
  // find all the paths that do not has collision with obstacles
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
      println("checking");
      if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10 && paths_status[i][j] == 0 ) {
        
        boolean feasible =  checkPath_CCD(sample_points[i].pos, sample_points[j].pos);
        
         if (feasible) {
           
           sample_points[i].neighbors.add(sample_points[j]);
           sample_points[j].neighbors.add(sample_points[i]);
           paths_status[i][j] = 1;
        
        }
      }

      
    }
  }
  reSearch();
  drawSP();
}

void reSearch(){
  println(searchLimit);
  if(searchLimit < 0) {
    searchLimit = 1000;
    return;
  }
  int allNoPath = 0;
  println("nopath1:" + allNoPath);
  for(int i = 0; i < agents.size(); i++){
    //agents.get(i).setGoalForce(new PVector(0,0));
    if(!agents.get(i).reachGoal)
    { 
      if(agents.get(i).myGoal != null) agents.get(i).searchPath(agents.get(i).pos,agents.get(i).myGoal.pos);
      if(agents.get(i).myPath.size()<1) allNoPath ++;
    }
  }
  
  if(allNoPath > 15) {
      println("nopath:" + allNoPath);
      resampleAll();
    }

}

void drawSP(){
    for (int i = 0; i < sample_number; i++) {
    fill(0,0,0);
    circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
    //text((int)sample_points[i].g+", "+(int)sample_points[i].h,sample_points[i].pos.x, sample_points[i].pos.y);
    
  }
}
