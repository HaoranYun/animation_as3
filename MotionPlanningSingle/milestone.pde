class Milestone{
  
  boolean isGoal = false;
  boolean isVisited = false;
  float g;
  float h;
  float f;
  PVector pos;
  
  ArrayList<Milestone> neighbors = new ArrayList<Milestone>();
  Milestone parent;

  
  Milestone(PVector position){
    pos = position;
  }
  
  //void getG(){
  //  g =  PVector.sub(pos,start).mag();
  //}
  
  //void getH(){
  //  h = PVector.sub(pos,goal).mag();
  //}
  
  void setG(float g_value){
    g = g_value;
    updateF();
  }
  void updateF(){
    f = g + h;
  }
  
  
  void reset( PVector start, PVector goal){
    g = PVector.sub(pos,start).mag();
    h = PVector.sub(pos,goal).mag();
    f = g + h;
    parent = null;
    isVisited = false;
  }

}


//Class for comparing f value of two milestones

class MyCompare implements Comparator<Milestone>{
    public int compare(Milestone m1, Milestone m2) {
        float f1 = m1.f;
        float f2 = m2.f;
        return Float.compare(f1,f2);
    }  
}
