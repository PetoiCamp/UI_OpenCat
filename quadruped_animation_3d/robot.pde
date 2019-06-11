import Jama.*;
import java.lang.*;

class robot
{
  
  Matrix torso, knee, toe;
  
  Matrix torso_transformed;
  Matrix knee_front_left_transformed, knee_front_right_transformed, knee_back_left_transformed, knee_back_right_transformed;
  Matrix toe_front_left_transformed, toe_front_right_transformed, toe_back_left_transformed, toe_back_right_transformed;
  
  double[] knee_position_front_left, knee_position_front_right, knee_position_back_left,knee_position_back_right;
  double[] toe_position;
  
  double[] torso_angles_, hip_angle_front_left_, hip_angle_front_right_, hip_angle_back_left_, hip_angle_back_right_, 
           knee_angle_front_left_, knee_angle_front_right_, knee_angle_back_left_, knee_angle_back_right_;
           
  double thigh_len_, foot_len_, torso_len_, torso_width_;
  
  robot(double torso_len, double torso_width, double thigh_len, double foot_len)
  {
    thigh_len_ = thigh_len;
    foot_len_ = foot_len;
    torso_len_ = torso_len;
    torso_width_ = torso_width;
    
    // end affector coords in local frames
    double[][] torso_coordinates = {{0.5*torso_len,   0.5*torso_len,    -0.5*torso_len,   -0.5*torso_len},
                                    {0.5*torso_width, -0.5*torso_width, 0.5*torso_width, -0.5*torso_width},
                                    {0.0,             0.0,              0.0,              0.0},
                                    {1.0d,            1.0d,             1.0d,             1.0d}};
    
    double[][] knee_coordinates = {{thigh_len},
                                    {0.0},
                                    {0.0},
                                    {1.0}};
                                    
    double[][] toe_coordinates = {{foot_len},
                                  {0.0},
                                  {0.0},
                                  {1.0}};    
   
    // position of coordinates w.r.t previous limb
    knee_position_front_left = new double[] {0.5*torso_len,0.5*torso_width,0.0};
    knee_position_front_right = new double[] {0.5*torso_len,-0.5*torso_width,0.0};
    knee_position_back_left = new double[] {-0.5*torso_len,0.5*torso_width,0.0};
    knee_position_back_right = new double[] {-0.5*torso_len,-0.5*torso_width,0.0};
    
    toe_position = new double[] {thigh_len,0.0,0.0};
    
    torso = new Matrix(torso_coordinates);
    knee = new Matrix(knee_coordinates);
    toe = new Matrix(toe_coordinates);
  }
  
  Matrix create_homogenous_transform(double[] angles, double[] position)
  {
    /*
    angles[0] = yaw
    angles[1] = pitch
    angles[2] = roll
    
    R = R(yaw)*R(pitch)*R(roll)
    
    position[0] = x
    position[1] = y
    position[2] = z
    */
    double alpha = Math.toRadians(angles[0]);
    double beta = Math.toRadians(angles[1]);
    double gamma = Math.toRadians(angles[2]);
    
    double [][] matrix = {{Math.cos(alpha)*Math.cos(beta), Math.cos(alpha)*Math.sin(beta)*Math.sin(gamma)-Math.sin(alpha)*Math.cos(gamma), Math.cos(alpha)*Math.sin(beta)*Math.cos(gamma)+Math.sin(alpha)*Math.sin(gamma), position[0]},
                          {Math.sin(alpha)*Math.cos(beta), Math.sin(alpha)*Math.sin(beta)*Math.sin(gamma)+Math.cos(alpha)*Math.cos(gamma), Math.sin(alpha)*Math.sin(beta)*Math.cos(gamma)-Math.cos(alpha)*Math.sin(gamma), position[1]},
                          {-Math.sin(beta),                Math.cos(beta)*Math.sin(gamma),                                                 Math.cos(beta)*Math.cos(gamma),                                                 position[2]},
                          {0,                              0,                                                                              0,                                                                              1}};
    Matrix ret = new Matrix(matrix);
    return ret;
  }
  
void update(double[][] angles)
  {   
    torso_angles_ = Angle_Matrix[0];
    
    //changing left<->right, back<->front..the double negative as talked about in the email
    hip_angle_front_left_ = angles[7];
    hip_angle_front_right_ = angles[5];
    hip_angle_back_left_ = angles[3];
    hip_angle_back_right_ = angles[1];
    knee_angle_front_left_ = angles[8];
    knee_angle_front_right_ = angles[6];
    knee_angle_back_left_ = angles[4];
    knee_angle_back_right_ = angles[2];
    
    Matrix torso_transformation = create_homogenous_transform(torso_angles_, Angle_Matrix[9]);

    
    Matrix knee_transformation_front_left = create_homogenous_transform(hip_angle_front_left_, knee_position_front_left);
    Matrix knee_transformation_front_right = create_homogenous_transform(hip_angle_front_right_, knee_position_front_right);
    Matrix knee_transformation_back_left = create_homogenous_transform(hip_angle_back_left_, knee_position_back_left);
    Matrix knee_transformation_back_right = create_homogenous_transform(hip_angle_back_right_, knee_position_back_right);
    
    knee_transformation_front_left = torso_transformation.times(knee_transformation_front_left);
    knee_transformation_front_right = torso_transformation.times(knee_transformation_front_right);
    knee_transformation_back_left = torso_transformation.times(knee_transformation_back_left);
    knee_transformation_back_right = torso_transformation.times(knee_transformation_back_right);
    
    Matrix toe_transformation_front_left = create_homogenous_transform(knee_angle_front_left_, toe_position);
    toe_transformation_front_left = knee_transformation_front_left.times(toe_transformation_front_left);
    
    Matrix toe_transformation_front_right = create_homogenous_transform(knee_angle_front_right_, toe_position);
    toe_transformation_front_right = knee_transformation_front_right.times(toe_transformation_front_right);
    
    Matrix toe_transformation_back_left = create_homogenous_transform(knee_angle_back_left_, toe_position);
    toe_transformation_back_left = knee_transformation_back_left.times(toe_transformation_back_left);
 
    Matrix toe_transformation_back_right = create_homogenous_transform(knee_angle_back_right_, toe_position);
    toe_transformation_back_right = knee_transformation_back_right.times(toe_transformation_back_right);
    
    torso_transformed = torso_transformation.times(torso);
    knee_front_left_transformed = knee_transformation_front_left.times(knee);
    toe_front_left_transformed = toe_transformation_front_left.times(toe);
    knee_front_right_transformed = knee_transformation_front_right.times(knee);
    toe_front_right_transformed = toe_transformation_front_right.times(toe);
    knee_back_left_transformed = knee_transformation_back_left.times(knee);
    toe_back_left_transformed = toe_transformation_back_left.times(toe);
    knee_back_right_transformed = knee_transformation_back_right.times(knee);
    toe_back_right_transformed = toe_transformation_back_right.times(toe); 
  }
  
  void display()
  { 

    //Combining all of the matrices from before into one
    double matrix[][][] = new double[9][][]; 
    
    matrix[0] = torso_transformed.getArrayCopy();
    matrix[1] = knee_front_left_transformed.getArrayCopy();
    matrix[2] = toe_front_left_transformed.getArrayCopy();
    matrix[3] = knee_front_right_transformed.getArrayCopy();
    matrix[4] = toe_front_right_transformed.getArrayCopy();
    matrix[5] = knee_back_left_transformed.getArrayCopy();
    matrix[6] = toe_back_left_transformed.getArrayCopy();
    matrix[7] = knee_back_right_transformed.getArrayCopy();
    matrix[8] = toe_back_right_transformed.getArrayCopy();
    
    for(int j=0;j<9;j++)
    {
      if(j==0)
      {      
            for(int i=0;i<4;i++)
              {
                pushMatrix();
                translate((float)matrix[0][0][i], (float)matrix[0][1][i], (float)matrix[0][2][i]);
                sphere(5);
                popMatrix();
              }
              fill(0,255,0);
              beginShape();
              //for(int i=1;i<4;i++){vertex((float)matrix[0][0][i], (float)matrix[0][1][i], (float)matrix[0][2][i]);}
              vertex((float)matrix[0][0][0], (float)matrix[0][1][0], (float)matrix[0][2][0]);
              vertex((float)matrix[0][0][2], (float)matrix[0][1][2], (float)matrix[0][2][2]);
              vertex((float)matrix[0][0][3], (float)matrix[0][1][3], (float)matrix[0][2][3]);
              vertex((float)matrix[0][0][1], (float)matrix[0][1][1], (float)matrix[0][2][1]);
              endShape(CLOSE);
              fill(0,0,0);
              pushMatrix();
              translate((0.6)*(float)((float)matrix[0][0][2]+matrix[0][0][3]), (0.6)*(float)((float)matrix[0][1][2]+matrix[0][1][3]), (0.6)*(float)((float)matrix[0][2][2]+matrix[0][2][3]));
              sphere(10);
              popMatrix();    
      }
      else if(j%2==0)
      {
        if(j==2||j==8)
        {
          fill(0,255,0);
          beginShape();
          vertex((float)((0.95)*matrix[j][0][0]+(0.05)*matrix[0][0][1]), (float)((0.95)*matrix[j][1][0]+(0.05)*matrix[0][1][1]), (float)((0.95)*matrix[j][2][0]+(0.05)*matrix[0][2][1]));
          vertex((float)matrix[j-1][0][0], (float)matrix[j-1][1][0], (float)matrix[j-1][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);
          
          beginShape();
          vertex((float)((0.95)*matrix[j][0][0]+(0.05)*matrix[0][0][2]), (float)((0.95)*matrix[j][1][0]+(0.05)*matrix[0][1][2]), (float)((0.95)*matrix[j][2][0]+(0.05)*matrix[0][2][2]));
          vertex((float)matrix[j-1][0][0], (float)matrix[j-1][1][0], (float)matrix[j-1][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);
        }
        
        else
        {
          fill(0,255,0);
          beginShape();
          vertex((float)((0.95)*matrix[j][0][0]+(0.05)*matrix[0][0][0]), (float)((0.95)*matrix[j][1][0]+(0.05)*matrix[0][1][0]), (float)((0.95)*matrix[j][2][0]+(0.05)*matrix[0][2][0]));
          vertex((float)matrix[j-1][0][0], (float)matrix[j-1][1][0], (float)matrix[j-1][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);

          beginShape();
          vertex((float)((0.95)*matrix[j][0][0]+(0.05)*matrix[0][0][3]), (float)((0.95)*matrix[j][1][0]+(0.05)*matrix[0][1][3]), (float)((0.95)*matrix[j][2][0]+(0.05)*matrix[0][2][3]));
          vertex((float)matrix[j-1][0][0], (float)matrix[j-1][1][0], (float)matrix[j-1][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);        
        }
      }
      else
      {
        if(j==1)
        {
          fill(255,0,0);
          beginShape();
          vertex((float)((0.85)*matrix[0][0][0]+(0.15)*matrix[0][0][1]), (float)((0.85)*matrix[0][1][0]+(0.15)*matrix[0][1][1]), (float)((0.85)*matrix[0][2][0]+(0.15)*matrix[0][2][1]));
          vertex((float)matrix[0][0][0], (float)matrix[0][1][0], (float)matrix[0][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);
          
          beginShape();
          vertex((float)((0.9)*matrix[0][0][0]+(0.1)*matrix[0][0][2]), (float)((0.9)*matrix[0][1][0]+(0.1)*matrix[0][1][2]), (float)((0.9)*matrix[0][2][0]+(0.1)*matrix[0][2][2]));
          vertex((float)matrix[0][0][0], (float)matrix[0][1][0], (float)matrix[0][2][0]);
          vertex((float)matrix[j][0][0], (float)matrix[j][1][0], (float)matrix[j][2][0]);
          endShape(CLOSE);        
        }
        else if(j==3)
        {
          fill(255,0,0);
          beginShape();
          vertex((float)((0.15)*matrix[0][0][0]+(0.85)*matrix[0][0][1]),(float)((0.15)*matrix[0][1][0]+(0.85)*matrix[0][1][1]), (float)((0.15)*matrix[0][2][0]+(0.85)*matrix[0][2][1]));
          vertex((float)matrix[0][0][1],(float)matrix[0][1][1],(float)matrix[0][2][1]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);
          
          beginShape();
          vertex((float)((0.1)*matrix[0][0][3]+(0.9)*matrix[0][0][1]),(float)((0.1)*matrix[0][1][3]+(0.9)*matrix[0][1][1]),(float)((0.1)*matrix[0][2][3]+(0.9)*matrix[0][2][1]));
          vertex((float)matrix[0][0][1],(float)matrix[0][1][1],(float)matrix[0][2][1]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);      
        }
        else if(j==5)
      {
          fill(255,0,0);
          beginShape();
          vertex((float)((0.85)*matrix[0][0][2]+(0.15)*matrix[0][0][3]),(float)((0.85)*matrix[0][1][2]+(0.15)*matrix[0][1][3]),(float)((0.85)*matrix[0][2][2]+(0.15)*matrix[0][2][3]));
          vertex((float)matrix[0][0][2],(float)matrix[0][1][2],(float)matrix[0][2][2]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);
          
          beginShape();
          vertex((float)((0.9)*matrix[0][0][2]+(0.1)*matrix[0][0][0]),(float)((0.9)*matrix[0][1][2]+(0.1)*matrix[0][1][0]),(float)((0.9)*matrix[0][2][2]+(0.1)*matrix[0][2][0]));
          vertex((float)matrix[0][0][2],(float)matrix[0][1][2],(float)matrix[0][2][2]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);      
      }
          if(j==7)
        {
        
          fill(255,0,0);
          beginShape();
          vertex((float)((0.15)*matrix[0][0][2]+(0.85)*matrix[0][0][3]),(float)((0.15)*matrix[0][1][2]+(0.85)*matrix[0][1][3]),(float)((0.15)*matrix[0][2][2]+(0.85)*matrix[0][2][3]));
          vertex((float)matrix[0][0][3],(float)matrix[0][1][3],(float)matrix[0][2][3]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);
          
          beginShape();
          vertex((float)((0.1)*matrix[0][0][1]+(0.9)*matrix[0][0][3]),(float)((0.1)*matrix[0][1][1]+(0.9)*matrix[0][1][3]),(float)((0.1)*matrix[0][2][1]+(0.9)*matrix[0][2][3]));
          vertex((float)matrix[0][0][3],(float)matrix[0][1][3],(float)matrix[0][2][3]);
          vertex((float)matrix[j][0][0],(float)matrix[j][1][0],(float)matrix[j][2][0]);
          endShape(CLOSE);        
        }
        
      }
    }
  }
}
