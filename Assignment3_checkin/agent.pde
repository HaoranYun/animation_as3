class Agent{
  PVector pos;
  PVector vel;
  float speed = 5;
  
  Agent(PVector position){
    pos = new PVector(position.x, position.y);
    println(position);
    println(pos);
    vel = new PVector(0,0);
  }
  
  void update(){
    pos.add(vel);
    if(straightDistance(path.get(idx).pos,pos) < 5){
      if(path.get(idx).isGoal) agent.setVel(new PVector(0,0,0)) ;
      else{
        agent.setVel(PVector.sub(path.get(idx+1).pos,path.get(idx).pos).normalize());
        idx ++;
      }
    }
  }
  
  void setVel(PVector dir){
    vel = PVector.mult(dir,speed);
  }
}
