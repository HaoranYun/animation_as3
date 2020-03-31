class Agent{
  PVector pos;
  PVector goalForce;
  float speed = 1;
  Milestone myGoal;
  Milestone myStart;
  ArrayList<Milestone> myPath = new ArrayList<Milestone>();
  int idx = 0;
  
  float forceRadius = 100;
  boolean reachGoal = false;
  float expectedDuration = 0;
  float startTime = 0;
  
  boolean runBFS= false;
  
  
  float dt = 0.5;
  PVector vel = new PVector(0,0);
  
  float r;
  PVector force = new PVector(0,0);
  
  Agent(PVector position, float radius){
    pos = new PVector(position.x, position.y);
    goalForce = new PVector(0,0);
    r = radius;
  }
  
  void update(){
    if(myPath.size() <1) return;
    
    if( !reachGoal&& PVector.sub(myGoal.pos, pos).mag() < 15) {
        searchPath(pos,myGoal.pos);
        return;
     }
    float distForward = straightDistance(myPath.get(idx).pos,pos);
    if(distForward < 30 && idx < myPath.size() -1){
        setGoalForce(PVector.sub(myPath.get(idx+1).pos,myPath.get(idx).pos));
        idx ++;
    }
    
    goalForce = PVector.sub(myPath.get(idx).pos,pos).normalize().mult(5).sub(vel);


    if(force.mag() > 0)  vel.add(PVector.mult(force,dt));
    vel.add(PVector.mult(goalForce,dt));
    
    vel.limit(5);
    pos.add(PVector.mult(vel,dt));
    force = new PVector(0,0);    
  }
  
  
 
  void getForce(){
    PVector avoidance = new PVector(0,0);
  
    int count = 0;
    
      for(int i = 0; i < agents.size(); i++){
       Agent curr = agents.get(i);
       float dist = PVector.sub(pos, curr.pos).mag();
       if(dist < forceRadius && dist > 0){
          // only count neighbors
          avoidance.add(getAvoidanceForce(curr.pos, curr.vel, curr.r));
       }
    }
    
    for(int i = 0; i < obstacles.size(); i++){
      Obstacle curr = obstacles.get(i);
      avoidance.add(getAvoidanceForce(curr.center, new PVector(0,0), curr.r));
      
    }
    
    force.add(avoidance);
    force.limit(20);
  }
  
  PVector getAvoidanceForce(PVector thatPos, PVector thatVel, Float thatRadius){
    PVector avoidance = new PVector(0,0);
    float ttc =Float.MAX_VALUE;
    
    PVector posDiff =  PVector.sub(thatPos, pos);
    PVector velDiff = PVector.sub(vel,thatVel);
    float rSum = (thatRadius + r) ;
    
    float a = PVector.dot(velDiff,velDiff);
    float b = PVector.dot(posDiff,velDiff);
    float c = PVector.dot(posDiff,posDiff) - sq(rSum);
    
   
    
    float i = b*b - a* c;
    
    if(i > 0) ttc = (b - sqrt(i))/a;
    else return avoidance;
    
    float tempLimit = 100;
    if(ttc > 0 && ttc < tempLimit) {
      PVector myExpectedPos = PVector.add(pos, PVector.mult(vel,ttc));
      PVector neighborExpectedPos = PVector.add(thatPos, PVector.mult(thatVel,ttc));
      avoidance = PVector.sub(myExpectedPos,neighborExpectedPos).normalize();
    }
    float mag, maxMag ;
    maxMag = 1000;
    if(ttc > 0){
      mag= (tempLimit- ttc)/(ttc +0.001);
      if(mag > maxMag) mag = maxMag;
    }
    else{
      mag = maxMag;
    }
    avoidance.mult(mag);
    return avoidance.copy();
  }
  
  
  
  void setMyGoal(PVector goal){
    myGoal = new Milestone(goal.copy());
    myGoal.isGoal = true;
    searchPath(pos,myGoal.pos);
  }
  
  void setMyStart(PVector start){
    myStart = new Milestone(start);
  }
  
  void setGoalForce(PVector dir){
    expectedDuration = dir.mag()*5/dt/dt*1.5;
    startTime = 0;
  }
  
  
  void searchPath(PVector start, PVector goal){
    float s = millis();
    myPath = new ArrayList<Milestone>();
    idx = 0;
    myStart = new Milestone(start.copy());
    myGoal = new Milestone(goal.copy());
    myGoal.isGoal =true;
    myStart.reset(start,goal);
    myGoal.reset(start,goal);
    
    boolean feasible = checkPath_CCD(myGoal.pos, myStart.pos);
    vel = new PVector(0,0);
    
    
    if(feasible){
      myGoal.parent = myStart;
      buildPath();
      return;
    }
    
    
    for(int i = 0; i < sample_number; i ++){
      
      Milestone curr = sample_points[i];
      curr.reset(start,goal);
      if(runBFS) curr.h = 0;
      boolean feasibleStart = checkPath_CCD(myStart.pos, curr.pos);
      boolean feasibleGoal = checkPath_CCD(myGoal.pos, curr.pos);
    
      
      if (feasibleStart) {
        myStart.neighbors.add(curr);
        curr.neighbors.add(myStart);
      }
      if(feasibleGoal) {
        myGoal.neighbors.add(curr);
        curr.neighbors.add(myGoal);
      }
    }
    
    boolean runResult;
    if(runBFS){
      runResult = bfs();
    }
    else runResult = A_star();
    
    if(runResult){
      buildPath();
    }
  }
  
  boolean A_star(){
    PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new MyCompare());
  
  // add start point to the queue
    myStart.isVisited = true;
    fringe.add(myStart);
    
    int iter = 0;
    while (fringe.size() >0){
      Milestone curr = fringe.poll();
      iter++;

      if(curr.isGoal) {
        return true;
      }
      int num_neighbors = curr.neighbors.size();
      for(int i = 0; i < num_neighbors; i++){
        //println("i:" + i);
        Milestone child = curr.neighbors.get(i);
        if(!child.isVisited){
          child.parent = curr;
          child.g = curr.g + straightDistance(child.pos,curr.pos);
          child.isVisited = true;
          fringe.add(child);
        }
      }
   }
  
    return false;
  }
  
  void buildPath(){
    
    Milestone curr = myGoal;
    
    myPath = new ArrayList<Milestone>();
    idx = 0;
    //println("rebuild!");
    while(curr.parent!=null){
     myPath.add(curr);
     //println(curr.pos);
     curr = curr.parent;
    }
    myPath.add(curr);
    Collections.reverse(myPath);
    if(myPath.size()> 1){
    setGoalForce(PVector.sub(myPath.get(idx+1).pos,myPath.get(idx).pos));
    }
    idx++;
  }
  
  boolean bfs(){
 
   PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new MyCompare());
  
  // add start point to the queue
    myStart.isVisited = true;
    fringe.add(myStart);
    
    int iter = 0;
    while (fringe.size() >0){
      Milestone curr = fringe.poll();
      iter++;
      if(curr.isGoal) {
        return true;
     }
      int num_neighbors = curr.neighbors.size();
      for(int i = 0; i < num_neighbors; i++){
        //println("i:" + i);
        Milestone child = curr.neighbors.get(i);
        if(!child.isVisited){
          child.parent = curr;
          child.g = curr.g + straightDistance(child.pos,curr.pos);
          child.isVisited = true;
          fringe.add(child);
        }
      }
   }
  
    return false;
  }
}
