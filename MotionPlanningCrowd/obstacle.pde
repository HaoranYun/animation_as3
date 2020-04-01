class Obstacle{
  
  float r;
  PVector center;
  
  Obstacle(float radius, float x, float y, float z){
    r = radius;
    center = new PVector(x,y, z);
  }
}
