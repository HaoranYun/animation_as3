import java.util.*; 


float startTime;
int sample_number = 30;
float obstacle_radius = 50;
float character_radius = 7.5;
Milestone[] sample_points = new Milestone[sample_number];
int[][] paths_status = new int[sample_number+2][sample_number+2];

PVector movingGoal1 = new PVector(600,100);
PVector movingGoal2 = new PVector(600,100);



ArrayList<Agent> agents = new ArrayList<Agent>();
ArrayList<Obstacle> obstacles =new  ArrayList<Obstacle>();

int searchLimit = 1000;

// all boolean values that control the simulation behaviors
boolean SHOWPATH = true;// defualt: show path
boolean MOVING = false; // default: don't move until key ENTER is pressed
boolean MOVE_GOAL = false; // defualt: when direction keys are pressed, obstacle move



boolean TTC = true;
boolean ODD_BEHAVIOR = false;

boolean COMPARE_INTERACTION_MODE = false;
int AgentsInCompareMode = 20;

boolean COMPARE_NAVIGATION_MODE = false;
boolean SMOOTH = COMPARE_NAVIGATION_MODE;

int runTime = 0;

Camera3 g_cam;
int g_flags[] = new int[12];


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
    if(distance < ( obstacle_radius + character_radius)) result = false;
    
  }
  return result;
}


// Check if the path between two feasible sample point would has
// collision with obstacle.
// return false if collisions occurs; return true otherwise. 
// Just like the continuous collision detection (ray-sephera intersection )
boolean checkPath_CCD(PVector p1, PVector p2){
  
  float a,b,c;
  float i,mu1, mu2;
  PVector dp,center;
  
  float r;
 
  boolean feasible = true;
        
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    r = obstacle_radius + character_radius;
    
    
    dp = PVector.sub(p2,p1);
    a = dp.x*dp.x + dp.y * dp.y;
    b = 2*(dp.x*(p1.x - center.x) +dp.y*(p1.y - center.y));
    c = PVector.dot(center,center);
    c += PVector.dot(p1,p1);
    c -= 2* PVector.dot(center,p1);
    c -= r*r;
    i = b*b - 4*a*c;
  
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
  }
    return true;
}


void search(){
 
  if(searchLimit < 0) {
    println("Re-sample 1000 times but no path has been found.");
    //searchLimit = 1000;
    return;
  }
  
  
 if(COMPARE_NAVIGATION_MODE) {
      if(agents.get(0).myGoal != null)agents.get(0).searchRRTPath(agents.get(0).pos, agents.get(0).myGoal.pos);
      if(agents.get(1).myGoal != null) agents.get(1).searchPath(agents.get(1).pos,agents.get(1).myGoal.pos);
      if(agents.get(1).myPath.size()<1) sampleAllPoints();
      return;
  }
  
  int allNoPath = 0;
  
  
  for(int i = 0; i < agents.size(); i++){
    //agents.get(i).setGoalForce(new PVector(0,0));
    if(!agents.get(i).reachGoal)
    { 
      agents.get(i).searchPath(agents.get(i).pos,movingGoal1);
      if(agents.get(i).myPath.size()<1) allNoPath ++;
    }
  }
  
  for(int i = 20; i < agents.size(); i++){
    if(!agents.get(i).reachGoal)
    { 
      agents.get(i).searchPath(agents.get(i).pos,movingGoal2);
      if(agents.get(i).myPath.size()<1) allNoPath ++;
    }
  
  }
  // if half agents could not find path, we will re-sample points
  int limit = 15;
  
  if(allNoPath > limit) {
    sampleAllPoints();
    //search();
  }
  


}



void sampleAllPoints(){
  
   searchLimit--;
   if(searchLimit < 0) {
     println("Re-sample 1000 times but no path has been found.");
     return;
   }
  
  PVector sample_point;
  
  // generate random sample points
  for (int i = 0; i < sample_number; i++) {
    sample_point = new PVector(random(55,595), random(55,595));
    
    while (isFeasible(sample_point) == false) {
      sample_point = new PVector(random(55,595), random(55,595));
    }
    
    sample_points[i] = new  Milestone(sample_point);
  }
  
  // intialize all paths status as zero(no path)
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
      paths_status[i][j] = 0;
    }
  }
  
  // find all the paths that do not has collision with obstacles
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
      
      if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10 && paths_status[i][j] == 0) {
        boolean feasible =  checkPath_CCD(sample_points[i].pos, sample_points[j].pos);   
         if (feasible) {     
           sample_points[i].neighbors.add(sample_points[j]);
           sample_points[j].neighbors.add(sample_points[i]);
           paths_status[i][j] = 1;
           paths_status[j][i] = 1;
        }
      }
    }
  }
  search();
}



void sampleSomePoints(){
  //if(COMPARE_NAVIGATION_MODE)return;
    for (int i = 0; i < sample_number; i++) {
    if(!isFeasible(sample_points[i].pos)){
      PVector sample_point = new PVector(random(55,595), random(55,595));
      while (isFeasible(sample_point) == false) {
        sample_point = new PVector(random(55,595), random(55,595));
      }
      sample_points[i] = new Milestone(sample_point);
      sample_points[i].neighbors = new ArrayList<Milestone>();
      
      for (int j = 0; j < sample_number; j++) {
        paths_status[i][j] = 0;
        paths_status[j][i] = 0;
        if ( straightDistance(sample_points[i].pos, sample_points[j].pos) > 10) {
          boolean feasible =  checkPath_CCD(sample_points[i].pos, sample_points[j].pos);
           if (feasible) { 
             sample_points[i].neighbors.add(sample_points[j]);
             sample_points[j].neighbors.add(sample_points[i]);
             paths_status[i][j] = 1;
             paths_status[j][i] = 1;
          }
        }
      }
    }
  }
  
  searchLimit = 1000;
  search();
}




int countReachedAgent(){
  int count = 0;
  for(int i = 0; i < agents.size(); i ++){
    if(agents.get(i).reachGoal) count++;
  }
  return count;
}

void setupCompareInteractionMode(){
  movingGoal1 = new PVector(600,300);
  movingGoal2 = new PVector(600,300);
  for(int i = 0; i< 4; i++){
    obstacles.add(new Obstacle(obstacle_radius, 400, 100 + 100 *i));
  }

  for(int i = 0; i< 2; i++){
    //obstacles.add(new Obstacle(obstacle_radius, 520, 400));
    obstacles.add(new Obstacle(obstacle_radius, 400, 130 + 100 *(i+4)));
  }

  // set up two group of agents
  for(int i = 0; i < AgentsInCompareMode; i ++){
    agents.add(new Agent(new PVector(80 + 20*(i%5), 300 + 20* ((int)i/5 )),character_radius));
  }
}




void setupCommonMode(){
  movingGoal2 = new PVector(100,300);
  for(int i = 0; i< 4; i++){
    obstacles.add(new Obstacle(obstacle_radius, 400, 100 + 100 *i));
  }

  for(int i = 0; i< 1; i++){
    obstacles.add(new Obstacle(obstacle_radius, 520, 400));
  }

  // set up two group of agents
  for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(80 + 20*(i%5), 300 + 20* ((int)i/5 )),character_radius));
  }
  for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(400 + 20*((int)i/5), 500 + 20* ((int)i%5 )),character_radius));
  }
}


void  setupCompareNavigationMode(){
  for(int i = 0; i< 4; i++){
    obstacles.add(new Obstacle(obstacle_radius, 400, 100 + 100 *i));
  }

  for(int i = 0; i< 1; i++){
    obstacles.add(new Obstacle(obstacle_radius, 520, 400));
  }
  for(int i = 0; i < 2; i ++){
    agents.add(new Agent(new PVector(80, 300),character_radius));
  }
}

void checkCompareResult(){
    if(countReachedAgent() > 0.94 * agents.size() && runTime < 2){
    runTime++;
    if(TTC)print("TTC TIME: ");
    else print("BOIDS TIME: ");
    print(millis()- startTime);
    println(" ");
    if(runTime > 1) return;
    agents = new ArrayList<Agent>();
    for(int i = 0; i < 20; i ++){
      agents.add(new Agent(new PVector(80 + 20*(i%5), 300 + 20* ((int)i/5 )),character_radius));
    }
    TTC = !TTC;
    startTime = millis();
    search();
  }
}

void checkNavigationCompareResult(){
  println("Run 100 times PRM/RRT and calculate the average time.");
  float startTime2 = millis();
  for(int i = 0; i < 20; i ++){
    agents.get(1).searchPath(agents.get(0).pos, movingGoal1);
  }
  if( agents.get(1).myPath.size()<1) println("No path for the agent with PRM");
  println("PRM running Time: "+((millis() -startTime2))/100);
  
  
 startTime = millis();
 for(int i = 0; i < 20; i ++){
    agents.get(0).searchRRTPath(agents.get(0).pos, movingGoal1);
  }
  println("RRT running Time: "+((millis() -startTime))/100);

}

void setup() {
  
  //size(700,700);
  size(700,700, P3D);
  g_cam = new Camera3();
  
  // set up obstacles 
  if(COMPARE_INTERACTION_MODE){
    setupCompareInteractionMode();
  }
  else if(COMPARE_NAVIGATION_MODE){
    setupCompareNavigationMode();
  }
  else{
    setupCommonMode();
  }

  sampleAllPoints();
  
  startTime = millis();
  if(COMPARE_NAVIGATION_MODE) checkNavigationCompareResult();
  //else search();
  
  
}

void draw() {
  background(255,255,255);
  
  g_cam.MoveInLocalSpace(new PVector(g_flags[0] * 0.8, 0, 0));
  g_cam.MoveInLocalSpace(new PVector(0, g_flags[1] * 0.8, 0));
  g_cam.MoveInLocalSpace(new PVector(0, 0, g_flags[2] * 0.8));

  g_cam.RotateAlongLocalAxis(new PVector(1, 0, 0), g_flags[3] * 0.03);
  g_cam.RotateAlongLocalAxis(new PVector(0, 1, 0), g_flags[4] * 0.03);
  g_cam.RotateAlongLocalAxis(new PVector(0, 0, 1), g_flags[5] * 0.03);
  g_cam.Apply();
  
  //fill(255,255,255);
  //rect(50,50,600,600); // rectangle
  
  
  // draw all obstacles
  fill(0,0,255);
  for(int i = 0; i< obstacles.size(); i ++){
    fill(0,0,255);
    //circle(obstacles.get(i).center.x,obstacles.get(i).center.y, obstacles.get(i).r * 2);
    pushMatrix();
    noStroke();
    fill(0,0,255);

    translate(obstacles.get(i).center.x, obstacles.get(i).center.y, 0);
    sphere(obstacles.get(i).r);
    popMatrix();
    
  }


  //draw all nodes 
  for (int i = 0; i < sample_number; i++) {
    fill(0,0,0);
    //circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
    pushMatrix();
    noStroke();
    fill(0,0,0);

    translate(sample_points[i].pos.x, sample_points[i].pos.y, 0);
    sphere(5);
    popMatrix();
  }
  
  
  fill(200,200,20);
  stroke(200,200,20);


  for(int i = 0; i < agents.size(); i ++){
      if(MOVING) agents.get(i).getForce();
  }
  
  for(int i = 0; i < agents.size(); i ++){
      if(MOVING) agents.get(i).update();
      stroke(255,0,0);
      //circle(agents.get(i).pos.x, agents.get(i).pos.y, 15);
      pushMatrix();
      noStroke();
      fill(255,150,50);

      translate(agents.get(i).pos.x, agents.get(i).pos.y, 0);
      sphere(5);
      popMatrix();
      if(SHOWPATH && agents.get(i).myPath.size()>0 ){
        for(int j = 1; j < agents.get(i).myPath.size(); j++){
          pushMatrix();
          //noStroke();
          strokeWeight(10);
          fill(255,0,0);
          //line(agents.get(i).myPath.get(j-1).pos.x,agents.get(i).myPath.get(j-1).pos.y,0,agents.get(i).myPath.get(j).pos.x,agents.get(i).myPath.get(j).pos.y,0);
          translate(agents.get(i).myPath.get(j).pos.x, agents.get(i).myPath.get(j).pos.y, 0);
          sphere(5);
          popMatrix();
          //line(agents.get(i).myPath.get(j-1).pos.x,agents.get(i).myPath.get(j-1).pos.y,agents.get(i).myPath.get(j).pos.x,agents.get(i).myPath.get(j).pos.y);
        }
      }
  }
  
  if(COMPARE_INTERACTION_MODE) checkCompareResult();
  
  pushMatrix();
  noStroke();
  fill(0,255,255);

  translate(agents.get(0).myGoal.pos.x, agents.get(0).myGoal.pos.y, 0);
  sphere(10);
  popMatrix();
  if(!(COMPARE_INTERACTION_MODE || COMPARE_NAVIGATION_MODE)) 
  {
    pushMatrix();
    noStroke();
    fill(0,255,0);
    translate(agents.get(20).myGoal.pos.x, agents.get(20).myGoal.pos.y, 0);
    sphere(10);
    popMatrix();
  }
  

  
  fill(255,0,0);
  text("Please press ENTER to check agent's movement.",50,30);
  
}

void keyPressed(){
  
  PVector moveDirection = new PVector (0,0,0);
  
  if(keyCode == ENTER){
    MOVING = !MOVING;
  }
  else if (keyCode == TAB){ // SWITCH CONTROL 
    MOVE_GOAL = !MOVE_GOAL;
  }
  else if (keyCode == UP){
    moveDirection = new PVector(0,-15);
  }
  else if (keyCode == DOWN){
    moveDirection = new PVector(0,15);
  }
  else if (keyCode == LEFT){
    moveDirection = new PVector(-15,0);
  }
  else if (keyCode == RIGHT){
    moveDirection = new PVector(15,0);
  }
  else if (key == 'w') g_flags[1] = -10; // (-Z)
  else if (key == 's') g_flags[1] =  10; // (+Z)
  else if (key == 'a') g_flags[0] = -10; // (-X)
  else if (key == 'd') g_flags[0] =  10; // (+X)
  else if (key == 'q') g_flags[2] =  10; // (+Y)
  else if (key == 'e') g_flags[2] = -10; // (-Y)
  else if (key == 't') g_flags[3] =  1; // 
  else if (key == 'g') g_flags[3] = -1; // 
  else if (key == 'f') g_flags[4] =  1; // 
  else if (key == 'h') g_flags[4] = -1; // 
  else if (key == 'r') g_flags[5] = -1; // 
  else if (key == 'y') g_flags[5] =  1; // 
  else if (key == 'l') print(g_cam.GetStatusString());
  
  if(!MOVE_GOAL) {
    obstacles.get(obstacles.size()-1).center.add(moveDirection);
  }
  else{
    if(COMPARE_NAVIGATION_MODE){
      PVector newGoal1  = PVector.add(moveDirection,agents.get(0).myGoal.pos);
      if(isFeasible(newGoal1)){
        agents.get(0).setMyGoalRRT(PVector.add(moveDirection,agents.get(0).myGoal.pos));
        agents.get(1).setMyGoal(PVector.add(moveDirection,agents.get(1).myGoal.pos));
      }
    }
    else{
      for(int i = 0; i < agents.size(); i ++){
        agents.get(i).setMyGoal(PVector.add(moveDirection,agents.get(i).myGoal.pos));
      }
    }
  }
  if(moveDirection.mag() == 0) return;
  if(keyCode != ENTER){
    if(MOVE_GOAL) {
      searchLimit = 1000;
      search();
    }
    else sampleSomePoints();
  }
}

void keyReleased() {
  if      (key == 'w' || key == 's') { g_flags[1] = 0; }
  else if (key == 'a' || key == 'd') { g_flags[0] = 0; }
  else if (key == 'q' || key == 'e') { g_flags[2] = 0; }
  else if (key == 't' || key == 'g') { g_flags[3] = 0; }
  else if (key == 'f' || key == 'h') { g_flags[4] = 0; }
  else if (key == 'r' || key == 'y') { g_flags[5] = 0; }
}

void mousePressed(){
  obstacles.add(new Obstacle(obstacle_radius, mouseX, mouseY));
  sampleSomePoints();
}
