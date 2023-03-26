package frc.robot.util;

public class ArmKinematics {
    
    double[][] jointAngles;
	boolean errorNotif;

//4 DOF Forward Kinematics
//Inputs:
    //[0] Pos() - J1, J2, J3, J4 current actual position data
    //[1] DOF - Degrees of freedom
    //[2] link() - Length from center of J(i) to J(i+1) data
    
//Outputs: 
    //[0] CartPos() - X, Y, Z actual position data
    //[1] Pos() - J1, J2, J3, J4 new actual position data

public static double[][] forwardKinematics4DOF(double[][] pos, int dof, double[] link) {
    double[][] cartPos = new double[dof + 1][4];

    for (int i = 0; i < dof+ 1; i++ ){

        
//  [- 1.0*sin(t1)*sin(t4) - cos(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)), 1.0*sin(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) - 1.0*cos(t4)*sin(t1), sin(t2 + t3)*cos(t1), d4*(cos(t1)*cos(t2)*sin(t3) + 1.0*cos(t1)*cos(t3)*sin(t2)) - 1.0*d2*sin(t1) - 1.0*d3*sin(t1)]
// [  1.0*cos(t1)*sin(t4) - cos(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), 1.0*cos(t1)*cos(t4) + 1.0*sin(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), sin(t2 + t3)*sin(t1), d4*(cos(t2)*sin(t1)*sin(t3) + 1.0*cos(t3)*sin(t1)*sin(t2)) + 1.0*d2*cos(t1) + 1.0*d3*cos(t1)]
// [                                                              -1.0*sin(t2 + t3)*cos(t4),                                                                      sin(t2 + t3)*sin(t4),         cos(t2 + t3),                                                                         d1 + d4*cos(t2 + t3)]
// [                                                                                      0,                                                                                         0,                    0,                                                                                          1.0]
 
    //X position
    cartPos[i][0] = link[4] *(Math.cos(pos[i][1])*Math.cos(pos[i][2])*Math.sin(pos[i][3]) + 1.0*Math.cos(pos[i][1])*Math.sin(pos[i][3])*Math.sin(pos[i][2])) - 1.0*link[2]*Math.sin(pos[i][1]) - 1.0*link[3]*Math.sin(pos[i][1]);
    //d4*(cos(t1)*cos(t2)*sin(t3) + 1.0*cos(t1)*cos(t3)*sin(t2)) - 1.0*d2*sin(t1) - 1.0*d3*sin(t1)]

   //Y position
    cartPos[i][1] = link[4] *(Math.cos(pos[i][2])*Math.sin(pos[i][1])*Math.sin(pos[i][3]) + 1.0*Math.cos(pos[i][3])*Math.sin(pos[i][1])*Math.sin(pos[i][2])) + 1.0*link[2]*Math.cos(pos[i][1]) + 1.0*link[3]*Math.cos(pos[i][1]);
    //d4*(cos(t2)*sin(t1)*sin(t3) + 1.0*cos(t3)*sin(t1)*sin(t2)) + 1.0*d2*cos(t1) + 1.0*d3*cos(t1)]

    //Z position
    cartPos[i][2] = link[1] + link[4]*Math.cos(pos[i][2] + pos[i][3]);
    //d1 + d4*cos(t2 + t3)
    cartPos[i][3] = 0.0;
}
    return cartPos;

}
}
//Forward Kinematic 3dof Matrix in Radians

// [cos(t1 + t2)*cos(t3), -1.0*cos(t1 + t2)*sin(t3),      sin(t1 + t2),      d3*sin(t1 + t2)]
// [sin(t1 + t2)*cos(t3), -1.0*sin(t1 + t2)*sin(t3), -1.0*cos(t1 + t2), -1.0*d3*cos(t1 + t2)]
// [             sin(t3),                   cos(t3),                 0,              d1 + d2]
// [                   0,                         0,                 0,                  1.0]
 

 //Forward Kinematic 4dof Matrix in Radians

//  [- 1.0*sin(t1)*sin(t4) - cos(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)), 1.0*sin(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) - 1.0*cos(t4)*sin(t1), sin(t2 + t3)*cos(t1), d4*(cos(t1)*cos(t2)*sin(t3) + 1.0*cos(t1)*cos(t3)*sin(t2)) - 1.0*d2*sin(t1) - 1.0*d3*sin(t1)]
// [  1.0*cos(t1)*sin(t4) - cos(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), 1.0*cos(t1)*cos(t4) + 1.0*sin(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), sin(t2 + t3)*sin(t1), d4*(cos(t2)*sin(t1)*sin(t3) + 1.0*cos(t3)*sin(t1)*sin(t2)) + 1.0*d2*cos(t1) + 1.0*d3*cos(t1)]
// [                                                              -1.0*sin(t2 + t3)*cos(t4),                                                                      sin(t2 + t3)*sin(t4),         cos(t2 + t3),                                                                         d1 + d4*cos(t2 + t3)]
// [                                                                                      0,                                                                                         0,                    0,                                                                                          1.0]
 

