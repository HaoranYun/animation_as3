class Agent{
  PVector pos;
  PVector goalForce;
  float speed = 1;
  Milestone myGoal;
  Milestone myStart;
  ArrayList<Milestone> myPath = new ArrayList<Milestone>();
  int idx = 0;
  
  float forceRadius = 40;
  boolean reachGoal = false;
  float expectedDuration = 0;
  float startTime = 0;
  
  boolean runBFS= false;
  
  
  float dt = 0.5;
  PVector vel = new PVector(0,0);
  
  Agent(PVector position){
    pos = new PVector(position.x, position.y);
    goalForce = new PVector(0,0);
  }
  
  void update(){
    if(myPath.size() <1) return;
    
    if( !reachGoal&& PVector.sub(myGoal.pos, pos).mag() < 15) {
        searchPath(pos,myGoal.pos);
        return;
     }
    // if(reachGoal){
    //   setGoalForce(PVector.sub(myGoal.pos, pos));
    // }
     
    float distForward = straightDistance(myPath.get(idx).pos,pos);
    if(distForward < forceRadius && idx < myPath.size() -1){
        setGoalForce(PVector.sub(myPath.get(idx+1).pos,myPath.get(idx).pos));
        idx ++;
    }
    
    goalForce = PVector.sub(myPath.get(idx).pos,pos).normalize().mult(5);
    PVector force = getForce();

    if(force == null) vel.add(PVector.mult(goalForce,dt));
    else{
      force.limit(1);
      //println("f:"+ force + " goalF"+goalForce);
      force.add(goalForce);
      vel.add(PVector.mult(force,dt));
    }
    
    vel.limit(3);
    pos.add(PVector.mult(vel,dt));

    

    
    for(int i = 0; i < agents.size(); i++){
      Agent curr = agents.get(i);
      float dist = PVector.sub(pos, curr.pos).mag();
      if (dist > 0 && dist < 15){
          pos.add(PVector.sub(pos, curr.pos).mult((15-dist)/dist));
       }
    }
    
    if(myPath.size() < 1 || reachGoal) return;
        for(int n = 0; n < obstacles.size(); n++){
      PVector dist = PVector.sub(pos, obstacles.get(n).center);
      if(dist.mag() < 57.5) pos.add(dist.copy().normalize().mult(60 - dist.mag()));
      
    }
    
    if(PVector.dot(vel,goalForce)/vel.mag()/goalForce.mag() < 0.6){
      println("here");
      searchPath(pos,myGoal.pos);
      update();
      
    }
    
    
    
  }
  
  PVector getForce(){
    
    PVector seperation  = new PVector(0,0);
    PVector cohesion  = new PVector(0,0);
    PVector alignment  = new PVector(0,0);
    PVector force  = new PVector(0,0);
    int count = 0;
    
      for(int i = 0; i < agents.size(); i++){
       Agent curr = agents.get(i);
       float dist = PVector.sub(pos, curr.pos).mag();
       if(dist < forceRadius && dist > 15){
          // only count neighbors
          count++;
          seperation.add(PVector.sub(curr.pos,pos).mult(10/(dist*dist)));
          alignment.add(curr.vel.copy());
          cohesion.add(curr.pos);
       }
       else if (dist > 0 && dist < 15){
          pos.add(PVector.sub(pos, curr.pos).mult((15-dist)/dist));
       }
       
  }
    
    if(count == 0) return null;
    else

    seperation.div(count);
    alignment.div(count);
    cohesion.div(count).mult(10);
    //pos.add(cohesion.normalize()).add(seperation);

    //println("s:"+seperation + " a:" +alignment + " c:"+ cohesion );
    force.add(alignment).add(seperation).add(cohesion).div(30);
    //force.limit(1);
    return force;
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
