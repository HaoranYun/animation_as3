static class Mat3 {
  public PVector m[]; 
  public Mat3() {
    m = new PVector[3]; 
    m[0] = new PVector(1, 0, 0);
    m[1] = new PVector(0, 1, 0);
    m[2] = new PVector(0, 0, 1);
  }
  static Mat3 mult(Mat3 a, Mat3 b) {
    Mat3 ret = new Mat3();
    for (int i=0; i<3; i++) {
      ret.m[i].x = a.m[0].x * b.m[i].x + a.m[1].x * b.m[i].y + a.m[2].x * b.m[i].z;
      ret.m[i].y = a.m[0].y * b.m[i].x + a.m[1].y * b.m[i].y + a.m[2].y * b.m[i].z;
      ret.m[i].z = a.m[0].z * b.m[i].x + a.m[1].z * b.m[i].y + a.m[2].z * b.m[i].z;
    }
    return ret;
  }
  // u means unit vector
  static Mat3 RotationMatrix(PVector u, float theta) {
    float ct = cos(theta), st = sin(theta);

    float r11 = ct + u.x * u.x * (1 - ct),
          r12 = u.x * u.y * (1 - ct) - u.z * st,
          r13 = u.x * u.z * (1 - ct) + u.y * st,
          r21 = u.y * u.x * (1 - ct) + u.z * st,
          r22 = ct + u.y * u.y * (1 - ct),
          r23 = u.y * u.z * (1 - ct) - u.x * st,
          r31 = u.z * u.x * (1 - ct) - u.y * st,
          r32 = u.z * u.y * (1 - ct) + u.x * st,
          r33 = ct + u.z * u.z * (1 - ct);
    Mat3 ret = new Mat3();
    ret.m[0].x = r11; ret.m[0].y = r21; ret.m[0].z = r31;
    ret.m[1].x = r12; ret.m[1].y = r22; ret.m[1].z = r32;
    ret.m[2].x = r13; ret.m[2].y = r23; ret.m[2].z = r33;
    return ret;
  }
  
  Mat3 Transpose() {
    Mat3 ret = new Mat3();
    ret.m[0].x = m[0].x; ret.m[0].y = m[1].x; ret.m[0].z = m[2].x;
    ret.m[1].x = m[0].y; ret.m[1].y = m[1].y; ret.m[1].z = m[2].y;
    ret.m[2].x = m[0].z; ret.m[2].y = m[1].z; ret.m[2].z = m[2].z;
    return ret;
  }
  
  PVector Mult(PVector x) {
    PVector ret = new PVector();
    ret.x = m[0].x * x.x + m[1].x * x.y + m[2].x * x.z;
    ret.y = m[0].y * x.x + m[1].y * x.y + m[2].y * x.z;
    ret.z = m[0].z * x.x + m[1].z * x.y + m[2].z * x.z;
    return ret;
  }
};




interface MyCamera {
  abstract void MoveInLocalSpace(PVector x);
  abstract void RotateAlongLocalAxis(PVector axis, float delta_theta);
};

class Camera3 implements MyCamera {
  public PVector pos;
  Mat3 orientation;
  
  Camera3() { 
    pos = new PVector(0, 0, 500);
    orientation = new Mat3();
  }
  void Apply() {

    PVector center = pos.copy();
    Mat3 o = orientation;
    center.add(PVector.mult(o.m[2], -1)); 
    
    PVector up = o.m[1];
    camera(pos.x, pos.y, pos.z, 
           center.x, center.y, center.z, 
           up.x, up.y, up.z);
  }
  void MoveInLocalSpace(PVector pos_delta) {
    pos.add(PVector.mult(orientation.m[0], pos_delta.x));
    pos.add(PVector.mult(orientation.m[1], pos_delta.y));
    pos.add(PVector.mult(orientation.m[2], pos_delta.z));
  }
  void RotateAlongLocalAxis(PVector axis, float delta_theta) {
    if (axis.equals(new PVector(1, 0, 0))) delta_theta *= -1; // 
    orientation = Mat3.mult(orientation, Mat3.RotationMatrix(axis, delta_theta));
  }
  String GetStatusString() {
    return String.format(
      "Camera Position Delta, X:%2d, Y:%2d, Z:%2d\n" +
      "Camera Rotation Delta: Y:%2d, X:%2d, Z:%2d",
      g_flags[0], g_flags[1], g_flags[2],
      g_flags[4], g_flags[3], g_flags[5]
    );
  }
};
