class Agent{
  PVector pos;
  PVector goalForce;
  float maxSpeed = 5;
  Milestone myGoal;
  Milestone myStart;
  ArrayList<Milestone> myPath = new ArrayList<Milestone>();
  int idx = 0;
  
  float neighborRadius = 100;
  float reachOrNotRadius = 30;
  boolean reachGoal = false;

  
  float dt = 0.5;
  PVector vel = new PVector(0,0);
  
  float r;
  PVector force = new PVector(0,0);
  
  PVector prevPos;
  
  
  //store RRT graph
  ArrayList<Milestone> myGraph;
  Agent(PVector position, float radius){
    pos = new PVector(position.x, position.y);
    goalForce = new PVector(0,0);
    r = radius;
    prevPos = pos;
    if(COMPARE_NAVIGATION_MODE) reachOrNotRadius = 5;
  }
  
  void update(){
    if(myPath.size() <1) {
      return;
    }
    
    if( !reachGoal&& PVector.sub(myGoal.pos, pos).mag() < 100) {
        reachGoal = true;
        return;
     }
     
    // check if the agent is stucked or not every 10 seconds
    if(second()%10 == 0) stuckChecker();
    
    
    if(myPath.size() <1) {
      idx = 0;
      return;
    }
    float segmentLen = straightDistance(myPath.get(idx).pos, myPath.get(idx-1).pos);
    float distForward = straightDistance(myPath.get(idx).pos,pos);
    if(SMOOTH && distForward/segmentLen< 0.4) smoothPath();
    else{
        if(distForward < reachOrNotRadius && idx < myPath.size() -1){
          idx ++;
        }
    }
    
    goalForce = PVector.sub(myPath.get(idx).pos,pos).normalize().mult(maxSpeed).sub(vel);
    if(COMPARE_NAVIGATION_MODE) force = new PVector(0,0,0);

    if(force.mag() > 0)  vel.add(PVector.mult(force,dt));
    vel.add(PVector.mult(goalForce,dt));
    
    vel.limit(maxSpeed);
    pos.add(PVector.mult(vel,dt));
    force = new PVector(0,0);   
    
    if(!TTC) collisionSolver();
  }
  
  void smoothPath(){
    if(idx == myPath.size() - 1) return;
    if(checkPath_CCD(pos,myPath.get(idx+1).pos )){
      idx ++;
       println("SMOOTH");
      smoothPath();
    }
    
  }
  void collisionSolver(){
    if(myPath.size() < 1 || reachGoal) return;
    
    for(int n = 0; n < obstacles.size(); n++){
      PVector dist = PVector.sub(pos, obstacles.get(n).center);
      if(dist.mag() < 58) pos.add(dist.copy().normalize().mult(59 - dist.mag()));
    }
    
  }
  void stuckChecker(){
    if(COMPARE_NAVIGATION_MODE) return;
    PVector dist = PVector.sub(pos,prevPos);
    if(dist.mag() < reachOrNotRadius){
      prevPos = pos;
      searchPath(pos,myGoal.pos);
    }
  }
  
 
  void getForce(){
    PVector avoidance = new PVector(0,0);
    PVector seperation  = new PVector(0,0);
    PVector cohesion  = new PVector(0,0);
    PVector alignment  = new PVector(0,0);
    
    int count = 0;
    if(TTC) neighborRadius = 100;
    else neighborRadius = 40;
    
      for(int i = 0; i < agents.size(); i++){
       Agent curr = agents.get(i);
       float dist = PVector.sub(pos, curr.pos).mag();
       if(dist < neighborRadius && dist > 0){
          
          // only count neighbors
          if(TTC) avoidance.add(getAvoidanceForce(curr.pos, curr.vel, curr.r,100));
          else{
            count++;
            seperation.add(PVector.sub(pos,curr.pos).normalize().div(dist*dist));
            alignment.add(curr.vel.copy());
            cohesion.add(curr.pos.copy());
          }

       }
    }
    
    
    for(int i = 0; i < obstacles.size(); i++){
      Obstacle curr = obstacles.get(i);
      float dist = PVector.sub(pos, curr.center ).mag();
      if(dist < 70 && TTC) avoidance.add(getAvoidanceForce(curr.center, new PVector(0,0), curr.r,100)); 
    }
    
    
    if(TTC) {
      force.add(avoidance);
      force.limit(20);
    }
    else{
      if(count == 0) ;
      else{

        seperation.div(count);
        seperation.normalize().mult(15).sub(vel).limit(15);
      
        alignment.div(count);
        alignment.normalize().mult(5).sub(vel).limit(5);
      
        cohesion.div(count);
        cohesion.normalize().mult(5).sub(vel).limit(5);
        force.add(seperation).add(alignment).add(cohesion);
        force.limit(5);
      }
    }
  }
  
  PVector getAvoidanceForce(PVector thatPos, PVector thatVel, Float thatRadius, float limit){
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
    
    if(ttc < 0) return avoidance;
    
    float ttcLimit = limit;
    if( ttc < ttcLimit) {
      PVector myExpectedPos = PVector.add(pos, PVector.mult(vel,ttc));
      PVector neighborExpectedPos = PVector.add(thatPos, PVector.mult(thatVel,ttc));
      avoidance = PVector.sub(myExpectedPos,neighborExpectedPos).normalize();
    }
    
    float mag = 0;
    float maxMag = 1000;
    if(ttc < ttcLimit){
      mag= (ttcLimit- ttc)/(ttc +0.001);
      if(mag > maxMag) mag = maxMag;
    }
    
    if(mag > maxMag) mag = maxMag;
    
    avoidance.mult(mag);
    return avoidance.copy();
  }
  
  
  
  void setMyGoal(PVector goal){
    myGoal = new Milestone(goal.copy());
    myGoal.isGoal = true;
    searchPath(pos,myGoal.pos);
  }
  
  void setMyGoalRRT(PVector goal){
    myGoal = new Milestone(goal.copy());
    myGoal.isGoal = true;
    searchRRTPath(pos,myGoal.pos);
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
    
    // reset every milestone for each agent 
    for(int i = 0; i < sample_number; i ++){
      
      Milestone curr = sample_points[i];
      curr.reset(start,goal);
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
    runResult = A_star();
    
    if(runResult){
      buildPath();
    }
  }
  
  boolean A_star(){
    PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new MyCompare());

    myStart.isVisited = true;
    fringe.add(myStart);

    while (fringe.size() >0){
      Milestone curr = fringe.poll();
      if(curr.isGoal) {
        return true;
      }
      int num_neighbors = curr.neighbors.size();
      for(int i = 0; i < num_neighbors; i++){
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
    while(curr.parent!=null){
     myPath.add(curr);;
     curr = curr.parent;
    }
    myPath.add(curr);
    Collections.reverse(myPath);
    idx++;
  }
  
  // RRT implementation
   void searchRRTPath(PVector start, PVector goal){
    myPath = new ArrayList<Milestone>();
    idx = 0;
    myStart = new Milestone(start.copy());
    myGoal = new Milestone(goal.copy());
    myGoal.isGoal =true;
    myStart.reset(start,goal);
    myGoal.reset(start,goal);
    vel = new PVector(0,0);
    myGraph = new ArrayList<Milestone>();
    
    BuildRRTPath();
  }
  
  void BuildRRTPath(){
    int count = 0;
    boolean feasible;
    //PVector 
    myGraph.add(myStart);
    
    while(true){
      count ++;
      if(count % 10 == 0){
        //check goal connection
        for(int i = 0; i < myGraph.size(); i++){
          if(checkPath_CCD( myGoal.pos,myGraph.get(i).pos))
          {
 
            myGoal.parent = myGraph.get(i);
            buildPath();
            
            return;
          }
          
        }
      }
      
      PVector randPnt = new PVector(random(55,595), random(55,595));
      float minDist = Float.MAX_VALUE;
      int minIdx = Integer.MAX_VALUE;
      for(int i = 0; i < myGraph.size(); i++){
        float dist = PVector.sub(myGraph.get(i).pos, randPnt).mag();
        if(dist < minDist) {
          minDist = dist;
          minIdx = i;
        }
      }
      
      Milestone nearestNeighbor = myGraph.get(minIdx);
      if(!checkPath_CCD(nearestNeighbor.pos, randPnt)){
        walkUntilNoCollision(nearestNeighbor.pos, randPnt);
      }
      
      Milestone newPnt = new Milestone(randPnt);
      newPnt.parent = nearestNeighbor;
      myGraph.add(newPnt);
      
    }
    
    
  }
  void walkUntilNoCollision(PVector NeighborPos, PVector randPnt){
    PVector dir = PVector.sub(NeighborPos,randPnt);
    float dist = dir.mag();
    dir.normalize();
    float step = 1;

    
    while(!checkPath_CCD(NeighborPos,randPnt)){
      randPnt.add( PVector.mult(dir,step));
      step += 1;
      if(step > dist|| randPnt.x < 55 || randPnt.y< 55|| randPnt.x > 595 || randPnt.y > 595){
        break;
      }
    }
  }
  
}
