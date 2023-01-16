/*
 *	Authors:		Geraldine Barreto
 *					Eduardo Nunes
 *	Version:		1.0
 *  License:        GNU General Public License v3.0
 *	
 *	Description:	Inverse kinematics for mechDOG robot
 */

import {robotJoint, comm} from './canvas.js';
import {COMport} from './communication.js';

var L1_MECHDOG = 52;
var L2_MECHDOG = 93.4;
var L3_MECHDOG = 90;
var FOOT_RAD_MECHDOG = 15;
var W_MECHDOG = 75/2;
var L_MECHDOG = 200/2;
var A_MECHDOG = 45;
var A_MECHDOG_JOG = 30;
var B_MECHDOG_DYNAMIC = 20;
var B_MECHDOG_STATIC = 20;
var CGX_MECHDOG_DYNAMIC = 40;
var CGX_MECHDOG_STATIC = 15;
var X_MECHDOG = 0;
var Z_MECHDOG = 0;
var Y_MECHDOG = 140;
var BALANCE_DISTANCE_MECHDOG = 30;

//Limits
var CGY_MECHDOG_LIMIT_MIN = 60;
var CGY_MECHDOG_LIMIT_MAX = 160;
var CGX_MECHDOG_LIMIT_MIN = -45;
var CGX_MECHDOG_LIMIT_MAX = 45;
var CGZ_MECHDOG_LIMIT_MIN = -30;
var CGZ_MECHDOG_LIMIT_MAX = 30;
var ROLL_MECHDOG_LIMIT_MIN = -20; //degrees
var ROLL_MECHDOG_LIMIT_MAX = 20;
var PITCH_MECHDOG_LIMIT_MIN = -20;
var PITCH_MECHDOG_LIMIT_MAX = 20;
var YAW_MECHDOG_LIMIT_MIN = -20;
var YAW_MECHDOG_LIMIT_MAX = 20;
var BALANCE_DISTANCE_MECHDOG = 30;

var ROTANGLE = 0.1745; //10 deg in radians

var StopWalk = 0;
var WalkForward = 360;
var WalkBackward = 180;
var WalkRight = 90;
var WalkLeft = 270;

var AngleRange = 60;

var Rotation_Dir = {
    CCW: -1,
    StopRotation: 0,
    CW: 1
};

var Foot_Trajectory = {
    Circular: 1,
    Square: 2
};

var Gait_Type = {
    Dynamic: 1,
    Static: 3
};

var Special_Moves = {
	UP: 0,
	SIT: 1,
	LAY: 2,
	PAW: 3,
	WIGGLE: 4,
	TINKLE: 5,
    STRETCH: 6,
    JOG_ON: 7,
	JOG_OFF: 8
};

class Leg {
    constructor(leg_id, offset_x, offset_z, robot) {
        this.a1 = 0;
        this.a2 = 0;
        this.a3 = 0;
        this.xg = 0;
        this.zg = 0;
        this.angle = [0, 0, 0];
        this.leg_ID = leg_id;
        this.rightLeg = false;
        this.foot_rad = 0;
        this.L1 = 0;
        this.L2 = 0;
        this.L3 = 0;

        this.L1 = L1_MECHDOG;
        this.L2 = L2_MECHDOG;
        this.L3 = L3_MECHDOG;
        this.foot_rad = FOOT_RAD_MECHDOG;
        
        //Right legs have an even ID
        if (this.leg_ID <= 2) this.rightLeg = true;
        else this.rightLeg = false;

        //Resting offsets
        switch(this.leg_ID){
            case 1:
                this.rest_pos_x = offset_x;
                this.rest_pos_z = offset_z;
                break;
            case 2:
                this.rest_pos_x = -offset_x;
                this.rest_pos_z = offset_z;
                break;
            case 3:
                this.rest_pos_x = offset_x;
                this.rest_pos_z = -offset_z;
                break;
            default:
                this.rest_pos_x = -offset_x;
                this.rest_pos_z = -offset_z;
        }
        this.new_rest_pos_x = this.rest_pos_x;
        this.new_rest_pos_z = this.rest_pos_z;

        //Last leg position
        this.xg = this.rest_pos_x;
        this.zg = this.rest_pos_z;

    }
    inverseKinematics(x, y, z, joint_angles) {
        var yf = y - this.foot_rad;
        var dz;
        
        // Lateral displacement
        if (this.rightLeg) dz = z;
        else dz = -z;
        var beta = Math.atan((this.L1+dz)/yf);
        var h = yf/Math.cos(beta);
        var gamma = Math.acos(this.L1/h);
        var abduction_angle = beta+gamma-Math.PI/2.0;
        var Yz = h*Math.sin(gamma);
        
        // Frontal displacement
        var theta = Math.atan(x/Yz);
        var Y = Yz/Math.cos(theta);

        // Height
        var knee_angle = Math.acos((Math.pow(this.L2,2)+Math.pow(this.L3,2)-Math.pow(Y,2))/(2.0*this.L3*this.L2));
        var rotation_angle = Math.acos((Math.pow(this.L2,2)+Math.pow(Y,2)-Math.pow(this.L3,2))/(2.0*Y*this.L2)) + theta;

        if (!isNaN(abduction_angle) && !isNaN(rotation_angle) && !isNaN(knee_angle)){
            joint_angles[this.leg_ID-1][0] = degrees(abduction_angle)*10;
            joint_angles[this.leg_ID-1][1] = degrees(rotation_angle)*10;
            joint_angles[this.leg_ID-1][2] = degrees(knee_angle)*10;
        }
    }
}

class Joints{
    static mechdog_joint_offsets = [0,745,155];
    static mechdog_joint_minmax = [[-450,-600,0],[450,600,1800]];

	constructor(robot){
		this.joint_angles = [
            [0,0,0],
            [0,0,0],
            [0,0,0],
            [0,0,0]
        ];
        this.joint_offsets = [0,0,0];
        this.joint_minmax = [[0,0,0],[0,0,0]];
        for (var joint = 0; joint < 3; joint++) {
            for (var joint = 0; joint < 3; joint++) {
                this.joint_offsets[joint] = Joints.mechdog_joint_offsets[joint];
                this.joint_minmax[0][joint] = Joints.mechdog_joint_minmax[0][joint];
                this.joint_minmax[1][joint] = Joints.mechdog_joint_minmax[1][joint];
            }
        }
    }
	moveServo(id){
        var leg = id/10;
        var joint = id - leg*10;
        leg--;
        joint--;
        
        var angle = joint_angles[leg][joint] - joint_offsets[joint];
        if (angle < this.joint_minmax[0][joint]) angle = this.joint_minmax[0][joint];
        if (angle > this.joint_minmax[1][joint]) angle = this.joint_minmax[1][joint];
    
        this.updateJoints(leg, joint, angle);
    }
	moveLeg(leg){
        var angle;
        var leg_id = leg.leg_ID-1;
        for (var joint = 0; joint < 3; joint++) {
            angle = this.joint_angles[leg_id][joint] - this.joint_offsets[joint];
            if (angle < this.joint_minmax[0][joint]) angle = this.joint_minmax[0][joint];
            if (angle > this.joint_minmax[1][joint]) angle = this.joint_minmax[1][joint];
            this.updateJoints(leg, joint, angle);
        }
    }
	moveBody(){
        var angle;
        for (var leg = 0; leg < 4; leg++) {
            for (var joint = 0; joint < 3; joint++) {
                angle = this.joint_angles[leg][joint] - this.joint_offsets[joint];
                if (angle < this.joint_minmax[0][joint]) angle = this.joint_minmax[0][joint];
                if (angle > this.joint_minmax[1][joint]) angle = this.joint_minmax[1][joint];
                this.updateJoints(leg, joint, angle);
            }
        }
    }
    updateJoints(leg, joint, angle){
        //Send command
        if(comm.selected == COMport.USB) comm.send("#" +  str(leg+1) + str(joint+1) + "D" + parseInt(angle) + "\r");

        //Check if direction is inverted
        if (joint > 0) angle = -angle/10;
        else angle = angle/10;

        //Update animation
        robotJoint[leg][joint].slider.value(angle);
        robotJoint[leg][joint].input.value(robotJoint[leg][joint].slider.value());
    }
}

class Body{
    constructor(robot){
        this.cgy_limits = [];
        this.cgx_limits = [];
        this.cgz_limits = [];
        this.roll_limits = [];
        this.pitch_limits = [];
        this.yaw_limits = [];

        this.w = W_MECHDOG;
        this.l = L_MECHDOG;
        this.cgy_limits[0] = CGY_MECHDOG_LIMIT_MIN;
        this.cgy_limits[1] = CGY_MECHDOG_LIMIT_MAX;
        this.cgx_limits[0] = CGX_MECHDOG_LIMIT_MIN;
        this.cgx_limits[1] = CGX_MECHDOG_LIMIT_MAX;
        this.cgz_limits[0] = CGZ_MECHDOG_LIMIT_MIN;
        this.cgz_limits[1] = CGZ_MECHDOG_LIMIT_MAX;
        this.roll_limits[0] = ROLL_MECHDOG_LIMIT_MIN;
        this.roll_limits[1] = ROLL_MECHDOG_LIMIT_MAX;
        this.pitch_limits[0] = PITCH_MECHDOG_LIMIT_MIN;
        this.pitch_limits[1] = PITCH_MECHDOG_LIMIT_MAX;
        this.yaw_limits[0] = YAW_MECHDOG_LIMIT_MIN;
        this.yaw_limits[1] = YAW_MECHDOG_LIMIT_MAX;
        this.cgx_dynamic_gait = CGX_MECHDOG_DYNAMIC;
        this.cgx_static_gait = CGX_MECHDOG_STATIC;
        this.cgy_std = Y_MECHDOG;
        this.cgz_std = Z_MECHDOG;
        this.foot_elevation = A_MECHDOG;
        this.jog_foot_elevation = A_MECHDOG_JOG;
        this.step_distance_dynamic = B_MECHDOG_DYNAMIC;
        this.step_distance_static = B_MECHDOG_STATIC;
        this.X = X_MECHDOG;
        this.Z = Z_MECHDOG;
        this.balance_distance = BALANCE_DISTANCE_MECHDOG;

        this.roll = 0;
        this.pitch = 0;
        this.yaw = 0;
        // Gait variables
        this.beta = Gait_Type.Dynamic;          //1 for dynamic gait and 3 for static
        this.points = 4;                        //Simplified trajectory has 2 point in air
        this.steps = (1+this.beta)*this.points; //Total of points for a period of the foot trajectory
        this.trajectory_type = Foot_Trajectory.Circular;
        
        this.director_angle = 0;
        this.move_state = 0;                    //0 = Stop moving
        this.rot_angle = 0;
        this.balancing = false;
        this.stopped = true;
        this.new_sp_move = Special_Moves.UP;
        this.sp_move = Special_Moves.UP;
        this.spm_state = 0;
        
        this.cont = 0;
        this.update_flag = false;
        this.new_director_angle = 0;
        this.new_move_state = StopWalk;         //0 = Stop moving; 1 = Moving
        this.new_rot_angle = Rotation_Dir.StopRotation;
        this.new_beta = Gait_Type.Dynamic;
        this.legs = [];

        //Initialization
        this.robotPostureInit();
        this.joints = new Joints();	
        for(var i = 0; i<4; i++){
            let leg = new Leg(i+1,this.X,this.Z,robot);
            this.legs.push(leg);
        }
    }
    robotPostureInit(){	
        if (this.beta == Gait_Type.Dynamic) {
            this.cgx = this.cgx_dynamic_gait;
            this.cgy = this.cgy_std;
            this.cgz = this.cgz_std;
            this.a = this.foot_elevation;
            this.b = this.step_distance_dynamic;
        } else {
            this.beta = this.Gait_Type.Static;
            this.cgx = this.cgx_static_gait;
            this.cgy = this.cgy_std;
            this.cgz = this.cgz_std;
            this.a = this.foot_elevation;
            this.b = this.step_distance_static;
        }
    }
    cgx_blocked(){
        if (this.beta == Gait_Type.Dynamic){
            this.cgx = CGX_MECHDOG_DYNAMIC;
        }else if (this.beta == Gait_Type.Static){
            this.cgx = CGX_MECHDOG_STATIC;
        }
    }
    getLegPos(leg_ID, foot_positions, leg_pos, mode) {
        // Convert degrees to radians
        var Acl, L, Zy = 0, Xy = 0, ypdif, yrdif, xb, zb, Z, Lx, Ly, Lz;

        //YAW
        // Calculate the X and Z displacement & eliminate leg offset
        var xft = foot_positions[0] + this.cgx;
        var zft = foot_positions[2] + this.cgz;
        var L1 = this.legs[0].L1;

        if (leg_ID == 4) {
            Acl = Math.atan((this.l - xft) / (this.w + L1 - zft));
            L = (this.w + L1 - zft) / Math.cos(Acl);
            Zy = (-L * Math.cos(Acl - this.yaw) + this.w) + L1;
            Xy = (-L * Math.sin(Acl - this.yaw) + this.l);
        } else if (leg_ID == 2) {
            Acl = Math.atan((this.l - xft) / (this.w + L1 + zft));
            L = (this.w + L1 + zft) / Math.cos(Acl);
            Zy = (L * Math.cos(Acl + this.yaw) - this.w) - L1;
            Xy = (-L * Math.sin(Acl + this.yaw) + this.l);
        } else if (leg_ID == 3) {
            Acl = Math.atan((this.l + xft) / (this.w + L1 - zft));
            L = (this.w + L1 - zft) / Math.cos(Acl);
            Zy = (-L * Math.cos(Acl + this.yaw) + this.w) + L1;
            Xy = (L * Math.sin(Acl + this.yaw) - this.l);
        } else if (leg_ID == 1){
            Acl = Math.atan((this.l+xft)/(this.w+L1+zft));
            L = (this.w+L1+zft)/Math.cos(Acl);          
            Zy = (L*Math.cos(Acl-this.yaw)-this.w) - L1;
            Xy = (L*Math.sin(Acl-this.yaw)-this.l);
        }
        
        if (leg_ID == 3 || leg_ID == 1) ypdif = -this.l*Math.sin(this.pitch);
        else ypdif = this.l*Math.sin(this.pitch);
        if (leg_ID == 2 || leg_ID == 1) yrdif = -this.w*Math.sin(this.roll);
        else yrdif = this.w*Math.sin(this.roll);

        //PITCH
        var yb = this.cgy + ypdif + yrdif - foot_positions[1];
        if (leg_ID == 3 || leg_ID == 1) xb = (this.l*(1-Math.cos(this.pitch)));
        else xb = (this.l*Math.cos(this.pitch)-this.l);
        xb = xb + Xy;
        var ax = Math.atan(xb/yb);
        var pitch_t = this.pitch + ax;
        var hx = yb/Math.cos(ax);
        yb = hx*Math.cos(pitch_t);
        var X = hx*Math.sin(pitch_t);
        
        //ROLL
        if (leg_ID == 2 || leg_ID == 1) zb = this.w-this.w*Math.cos(this.roll) + L1;
        else zb = this.w*Math.cos(this.roll)-this.w - L1;
        zb = zb + Zy;
        var az = Math.atan((zb)/yb);
        var yh = yb/Math.cos(az);
        var roll_t = this.roll + az;
        var Y = yh*Math.cos(roll_t);
        if (leg_ID == 2 || leg_ID == 1) Z = yh*Math.sin(roll_t) - L1;
        else Z = yh*Math.sin(roll_t) + L1;
        
        if (mode == 0){
            Lz = Z;
            Lx = X;
        }
        else{
            Lz = Z + this.cgz;
            Lx = X + this.cgx;
        }
        Ly = Y;
        leg_pos[0] = Lx;
        leg_pos[1] = Ly;
        leg_pos[2] = Lz;
    }
    update_traj(){
        this.update_flag = false;
        this.director_angle = radians(this.new_director_angle);
        this.move_state = this.new_move_state;
        this.jog_mode = this.new_jog_mode;
        if(this.jog_mode) this.move_state = 1;

        if(this.new_rot_angle == Rotation_Dir.CCW){
            this.rot_angle = -ROTANGLE;
        }
        else if(this.new_rot_angle == Rotation_Dir.CW){
            this.rot_angle = ROTANGLE;
        }
        else if(this.new_rot_angle == Rotation_Dir.StopRotation){
            this.rot_angle = Rotation_Dir.StopRotation;
        }
    
        this.legs[0].new_rest_pos_x = this.X;
        this.legs[0].new_rest_pos_z = this.Z;
    
        this.legs[1].new_rest_pos_x = -this.X;
        this.legs[1].new_rest_pos_z = this.Z;
    
        this.legs[2].new_rest_pos_x = this.X;
        this.legs[2].new_rest_pos_z = -this.Z;
    
        this.legs[3].new_rest_pos_x = -this.X;
        this.legs[3].new_rest_pos_z = -this.Z;
    }
    balance(){
        if(!this.balancing && (this.move_state != StopWalk || this.rot_angle != Rotation_Dir.StopRotation) ) this.balancing = true;
        if(this.balancing){
            this.cgz = parseInt((-1)*this.balance_distance*Math.sin(Math.PI*(this.cont-3)/8));
        }
    }
    trajectory(leg_ID, i, foot_pos){
        var rest_pos_update = false;
        var a = this.a;
        var b = this.b;
        if (this.move_state == 0) b = 0;
        if(this.jog_mode && this.director_angle == 0){
            a = A_MECHDOG_JOG;
            b = 0;
        }

        if (this.legs[leg_ID-1].rest_pos_x != this.legs[leg_ID-1].new_rest_pos_x || this.legs[leg_ID-1].rest_pos_z != this.legs[leg_ID-1].new_rest_pos_z){
            if(i == 0){
                this.legs[leg_ID-1].rest_pos_x = this.legs[leg_ID-1].new_rest_pos_x;
                this.legs[leg_ID-1].rest_pos_z = this.legs[leg_ID-1].new_rest_pos_z;
            }
            rest_pos_update = true;
        }

        var xft = this.legs[leg_ID-1].rest_pos_x;
        var zft = this.legs[leg_ID-1].rest_pos_z;

        var z = (-1)*b*Math.sin(this.director_angle);
        var x = b*Math.cos(this.director_angle);

        var Acl, L, Zy = 0, Xy = 0;
        var L1 = this.legs[0].L1;

        // Rotation
        if (leg_ID == 4){
            Acl = Math.atan((this.l-xft)/(this.w+L1-zft));
            L = (this.w+L1-zft)/Math.cos(Acl);
            Zy = (-L*Math.cos(Acl-this.rot_angle)+this.w) + L1;
            Xy = (-L*Math.sin(Acl-this.rot_angle)+this.l);
        }
        else if (leg_ID == 2){
            Acl = Math.atan((this.l-xft)/(this.w+L1+zft));
            L = (this.w+L1+zft)/Math.cos(Acl);
            Zy = (L*Math.cos(Acl+this.rot_angle)-this.w) - L1;
            Xy = (-L*Math.sin(Acl+this.rot_angle)+this.l);
        }
        else if (leg_ID == 3){
            Acl = Math.atan((this.l+xft)/(this.w+L1-zft));
            L = (this.w+L1-zft)/Math.cos(Acl);
            Zy = (-L*Math.cos(Acl+this.rot_angle)+this.w) + L1;
            Xy = (L*Math.sin(Acl+this.rot_angle)-this.l);
        }
        else if (leg_ID == 1){
            Acl = Math.atan((this.l+xft)/(this.w+L1+zft));
            L = (this.w+L1+zft)/Math.cos(Acl);
            Zy = (L*Math.cos(Acl-this.rot_angle)-this.w) - L1;
            Xy = (L*Math.sin(Acl-this.rot_angle)-this.l);
        }

        xft = x + (Xy - this.legs[leg_ID-1].rest_pos_x);
        zft = z + (Zy - this.legs[leg_ID-1].rest_pos_z);
        
        var stepx = xft/(this.beta*2); //beta: 3 stationary gait / 1 dynamic gait.
        var stepz = zft/(this.beta*2);
        
        var constant = Math.PI/this.points;
        var y = 0;

        if (i < this.points){                    //Foot on the air
            if (this.trajectory_type == Foot_Trajectory.Circular){
                y = a*Math.sin((i+1)*constant);
                x = xft*Math.cos((i+1)*constant);
                z = zft*Math.cos((i+1)*constant);
            }
            else if (this.trajectory_type == Foot_Trajectory.Square){
                if (i == 0){
                    y = a;
                    x = xft;
                    z = zft;
                }
                else if (i == 1){
                    y = a;
                    x = 0;
                    z = 0;
                }
                else if (i == 2){
                    y = a;
                    x = -xft;
                    z = -zft;
                }
                else if (i == 3){
                    y = 0;
                    x = -xft;
                    z = -zft;
                }
            }
            x += this.legs[leg_ID-1].rest_pos_x;
            z += this.legs[leg_ID-1].rest_pos_z;
            if (i == 3){
                this.legs[leg_ID-1].xg = x;
                this.legs[leg_ID-1].zg = z;
                y = 15;
            }
        }
        else{
            y = 0;
            this.legs[leg_ID-1].xg += stepx;
            this.legs[leg_ID-1].zg += stepz;
            x = this.legs[leg_ID-1].xg;
            z = this.legs[leg_ID-1].zg;
        }

        if(abs(this.legs[leg_ID-1].xg - this.legs[leg_ID-1].rest_pos_x)< 1 && abs(this.legs[leg_ID-1].zg - this.legs[leg_ID-1].rest_pos_z)<1 && (this.move_state == 0) && (this.rot_angle == 0)) {
            y = 0;
            //returned values
            foot_pos[0] = x;
            foot_pos[1] = y;
            foot_pos[2] = z;
            return true;
        }else{
            //returned values
            foot_pos[0] = x;
            foot_pos[1] = y;
            foot_pos[2] = z;
            return false;
        }
    }
    walk(){
        var is_posture_change = false;
	    if(this.new_move_state == 0 && this.new_rot_angle == Rotation_Dir.StopRotation && this.stopped) is_posture_change = true;
	
        var leg_order = [3,0,2,1];
        if (this.beta == Gait_Type.Static){
            leg_order[0] = 0;
            leg_order[1] = 1;
            leg_order[2] = 2;
            leg_order[3] = 3;
            this.balance();
        }

        // static to dynamic gait
        if (((this.cont == 3 || this.cont == 11) || this.stopped) && this.beta!=this.new_beta){
            this.beta = this.new_beta;
            this.steps = (1+this.beta)*this.points;
            this.cgx_blocked();
        }
        this.stopped = false;

        var leg_stopped = [this.stopped,this.stopped,this.stopped,this.stopped];

        var back_front = 0;
        var left_right = 0;
        if (this.update_flag){
            if ((WalkForward - this.new_director_angle) < AngleRange || this.new_director_angle < AngleRange){
                back_front = 1;
            }else if(abs(this.new_director_angle - WalkBackward) < AngleRange){
                back_front = -1;
            }
            if (abs(this.new_director_angle - WalkRight) < AngleRange){
                left_right = 1;
            }else if(abs(this.new_director_angle - WalkLeft) < AngleRange){
                left_right = -1;
            }
        }
        var i;
        for (var leg_number = 0; leg_number < 4; leg_number ++){
            i  = (this.cont+(leg_order[leg_number])*this.points)%this.steps;

            //Wait for the appropriate moment and leg before changing the trajectory of movement
            if (this.update_flag && i == 1){
                if ((back_front >= 0 && left_right >= 0 && leg_number == 1) || (this.new_rot_angle == Rotation_Dir.CW)){      //Front Right leg
                    this.update_traj();
                }
                else if ((back_front >= 0 && left_right <= 0 && leg_number == 3) || (this.new_rot_angle == Rotation_Dir.CCW)){ //Front Left leg
                    this.update_traj();
                }
                else if ((back_front <= 0 && left_right >= 0 && leg_number == 0) || (this.new_rot_angle == Rotation_Dir.CW)){ //Back Right leg
                    this.update_traj();
                }
                else if ((back_front <= 0 && left_right <= 0 && leg_number == 2) || (this.new_rot_angle == Rotation_Dir.CCW)){ //Back Left leg
                    this.update_traj();
                }
                if (this.new_move_state == StopWalk && this.new_rot_angle == Rotation_Dir.StopRotation){
                    this.update_traj();
                }
            }

            if(!is_posture_change){
                this.cgx_blocked();
            }
            var foot_pos = [0,0,0];
            var leg_pos = [0,0,0];
            leg_stopped[leg_number] = this.trajectory(leg_number+1, i,foot_pos);
            this.getLegPos(leg_number+1,foot_pos,leg_pos,0);
            this.legs[leg_number].inverseKinematics(leg_pos[0],leg_pos[1],leg_pos[2],this.joints.joint_angles);
        }

        if(leg_stopped[0]&&leg_stopped[1]&&leg_stopped[2]&&leg_stopped[3]){ 
            if(this.beta == Gait_Type.Static && (this.cgz==0 || is_posture_change) && this.new_rot_angle == Rotation_Dir.StopRotation && this.new_move_state == 0 && !this.update_flag){
                this.stopped = true;
                this.sp_move = this.new_sp_move;
                this.balancing = false;
            }else if(this.beta == Gait_Type.Dynamic && this.new_rot_angle == Rotation_Dir.StopRotation && this.new_move_state == 0){
                this.stopped = true;
                this.sp_move = this.new_sp_move;
            }
        }
        
        this.cont = this.cont + 1;
        if (this.cont >= this.steps) this.cont = 0;
    }
    specialMoves(){
        var y = 0;
        var mode = 0;
        var legs_updated = [false,false,false,false];
        switch(this.sp_move){
            case Special_Moves.SIT:
                switch(this.spm_state){
                    case 0: //SIT PART 1
                        this.stopped = false;
                        this.cgx = 0; 
                        this.cgz = 0;
                        this.pitch = 0;
                        this.yaw = 0;
                        this.roll = 0;
                        this.cgy = 120;
                        this.spm_state = 1;
                        break;
                    case 1: //SIT PART 2
                        this.pitch = radians(30);           
                        this.cgx = -40;
                        this.spm_state = 2;
                        break;
                    case 2: //MOVE PAW
                        this.spm_state = 3;
                        y = 60;
                        this.legs[3].zg = 20;
                        legs_updated[3] = true;
                        break;
                    case 3: //MOVE PAW
                        this.spm_state = 4;
                        y = 0;
                        legs_updated[3] = true;
                        break;
                    case 4: //MOVE PAW
                        this.spm_state = 5;
                        y = 60;
                        this.legs[1].zg = -20;
                        legs_updated[1] = true;
                        break;
                    case 5: //MOVE PAW
                        this.stopped = true;
                        y = 0;
                        legs_updated[1] = true;
                        if(this.new_sp_move != Special_Moves.SIT){
                            this.spm_state = 6;
                            this.stopped = false;
                        }
                        break;
                    case 6: //MOVE PAW
                        this.spm_state = 7;
                        y = 60;
                        this.legs[3].zg = this.legs[3].rest_pos_z;
                        legs_updated[3]=true;
                        break;
                    case 7:
                        this.spm_state = 8;
                        y = 0;
                        legs_updated[3]=true;
                        break;
                    case 8: 
                        this.spm_state = 9;
                        y = 60;
                        this.legs[1].zg = this.legs[1].rest_pos_z;
                        legs_updated[1]=true;
                        break;
                    case 9: 
                        this.spm_state = 10;
                        y = 0;
                        legs_updated[1]=true;
                        break;
                    case 10: 
                        this.pitch = 0;
                        this.cgx = 0;
                        this.spm_state = 11;
                        break;
                    case 11: 
                        this.cgy = Y_MECHDOG;
                        this.spm_state = 0;
                        this.sp_move = this.new_sp_move;
                        if(this.sp_move == Special_Moves.UP) this.stopped = true;
                        break;
                    }
                break;
            case Special_Moves.PAW:
                switch(this.spm_state){
                    case 0: 
                        this.stopped = false;
                        this.cgx = -10; 
                        this.cgy = 135; 
                        this.cgz = 20;
                        this.yaw = 0;
                        this.roll = 0;
                        y = 50;
                        this.pitch = radians(15);
                        legs_updated[1] = true;
                        this.spm_state = 1;
                        break;
                    case 1: 
                        y = Y_MECHDOG;
                        this.stopped = true;
                        this.legs[1].xg = -85;
                        this.legs[1].zg = -20;
                        legs_updated[1]=true;
                        if(this.new_sp_move != Special_Moves.PAW){
                            y = 50;
                            this.legs[1].xg = -20;
                            this.spm_state = 2;
                            this.stopped = false;
                        }
                        break;
                    case 2:
                        y = 0;
                        this.pitch = 0;
                        this.cgx = 0;
                        this.cgz = 0;
                        this.legs[1].xg = this.legs[1].rest_pos_x;
                        this.legs[1].zg = this.legs[1].rest_pos_z;
                        legs_updated[1]=true;
                        this.spm_state = 3;
                        break;
                    case 3: 
                        this.spm_state = 4;
                        break;
                    case 4: 
                        this.sp_move = this.new_sp_move;
                        this.spm_state = 0;
                        if(this.sp_move == Special_Moves.UP) this.stopped = true;
                        break;
                    }    
                break;
            case Special_Moves.LAY:
                switch(this.spm_state){
                    case 0:
                    this.stopped = false;
                    this.cgx = 0;
                    this.cgy = Y_MECHDOG;
                    this.cgz = 0;
                    this.pitch = 0;
                    this.yaw = 0;
                    this.roll = 0;
                    this.spm_state = 1;
                    break;
                    case 1:
                        this.stopped = true;
                        this.cgx = 0;
                        this.cgy = 60;
                        this.cgz = 0;
                        this.pitch = 0;
                        this.yaw = 0;
                        this.roll = 0;
                        if(this.new_sp_move != Special_Moves.LAY){
                            this.cgy = Y_MECHDOG;
                            this.spm_state = 2;
                            this.stopped = false;
                        }
                        break;
                    case 2: // time to stabilize
                        this.spm_state = 3;
                        break;
                    case 3: 
                        this.sp_move = this.new_sp_move;
                        this.spm_state = 0;
                        if(this.sp_move == Special_Moves.UP) this.stopped = true;
                        break;
                }
                break;
            case Special_Moves.TINKLE:
                switch(this.spm_state){
                    case 0: 
                        this.stopped = false;
                        this.cgx = 30; 
                        this.cgy = Y_MECHDOG; 
                        this.cgz = 30;
                        this.pitch = 0;
                        this.yaw = 0;
                        this.roll = 0;
                        this.spm_state = 1;
                    break;
                    case 1: 
                        y = Y_MECHDOG-70;
                        this.legs[0].zg = 30;
                        this.legs[0].xg = 0;
                        legs_updated[this.legs[0].leg_ID-1]=true;
                        this.spm_state = 2;
                        break;
                    case 2:
                        this.stopped = true; 
                        this.legs[0].zg = 120;
                        y = Y_MECHDOG-70;
                        legs_updated[this.legs[0].leg_ID-1]=true;
                        if(this.new_sp_move != Special_Moves.TINKLE){
                            this.legs[0].zg = 20;
                            this.spm_state = 3;
                            this.stopped = false;
                        }
                        break; 
                    case 3: 
                        this.spm_state = 4;
                        y = 0;
                        this.legs[0].zg = this.legs[0].rest_pos_z;
                        this.legs[0].xg = this.legs[0].rest_pos_x;
                        legs_updated[this.legs[0].leg_ID-1]=true;
                        this.cgy = Y_MECHDOG; 
                        break;
                    case 4: 
                        this.spm_state = 5;
                        this.cgz = 0;
                        this.cgx = 0;
                        this.cgy = Y_MECHDOG; 
                        break; 
                    case 5: //time to stabilize
                        this.spm_state = 6;
                        break;
                    case 6: 
                        this.spm_state = 0;
                        this.sp_move = this.new_sp_move;
                        if(this.sp_move == Special_Moves.UP) this.stopped = true;
                        break;
                    }
                break;
            case Special_Moves.WIGGLE:
                switch(this.spm_state){
                case 0: 
                    this.stopped = false;
                    this.cgx = -20; 
                    this.cgz = 0;
                    this.pitch = radians(-13);
                    this.yaw = 0;
                    this.roll = 0;
                    this.cgy = 135; 
                    this.spm_state = 1;
                    break;
                case 1:
                    mode = 1;
                    this.yaw = radians(10);
                    this.spm_state = 2;
                    break;
                case 2:
                    mode = 1;
                    this.yaw = radians(-10);
                    this.spm_state = 1;
                    if(this.new_sp_move != Special_Moves.WIGGLE){
                        this.yaw = 0;
                        this.spm_state = 3;
                    }
                    break;
                case 3:
                    this.pitch = 0;
                    this.spm_state = 4;
                    break;
                case 4:
                    this.cgx = 0; 
                    this.cgz = 0;
                    this.cgy = Y_MECHDOG; 
                    this.spm_state = 5;
                    break;
                case 5: //time to stabilize
                    this.spm_state = 6;
                    break;
                case 6:
                    this.spm_state = 0;
                    this.sp_move = this.new_sp_move;
                    if(this.sp_move == Special_Moves.UP) this.stopped = true;
                    break;
                }
                break;
            case Special_Moves.STRETCH:
                switch(this.spm_state){
                    case 0: 
                        this.stopped = false;
                        this.cgx = 0; 
                        this.cgz = 0;
                        this.pitch = 0;
                        this.yaw = 0;
                        this.roll = 0;
                        this.cgy = Y_MECHDOG; 
                        this.spm_state = 1;
                        break;
                    case 1: 
                        this.cgx = -20; 
                        this.cgy = 110; 
                        this.spm_state = 2;
                        break;
                    case 2: 
                        this.cgx = -50; 				
                        this.pitch = radians(-14);
                        this.cgy = 120; 
                        this.spm_state = 3;
                    case 3: 
                        this.yaw = 0;
                        this.roll = 0;
                        this.spm_state = 7;
                        break;
                    case 4: 
                        this.yaw = 5;
                        this.roll = 5;
                        this.spm_state = 5;
                        break;
                    case 5: 
                        this.yaw = 0;
                        this.roll = 0;
                        this.spm_state = 6;
                        break;
                    case 6: 
                        this.yaw = -5;
                        this.roll = -5;
                        this.spm_state = 7;
                        break;	
                    case 7: 
                        this.yaw = 0;
                        this.roll = 0;
                        this.stopped = true;
                        if(this.new_sp_move != Special_Moves.STRETCH){
                            this.pitch = 0;
                            this.cgx = -20; 
                            this.spm_state = 8;
                            this.stopped = false;
                            }
                        break;	
                    case 8: 
                        this.cgx = 0; 
                        this.cgz = 0;
                        this.yaw = 0;
                        this.cgy = Y_MECHDOG; 
                        this.spm_state = 9;
                        break;	
                    case 9: 
                        this.spm_state = 0;
                        this.sp_move = this.new_sp_move;
                        if(this.sp_move == Special_Moves.UP) this.stopped = true;
                        break;	
                }
                break;
            default:
                    this.new_sp_move = Special_Moves.UP;
                    this.sp_move = Special_Moves.UP;
        }
        var foot_pos = [0,0,0];
        var leg_pos = [0,0,0];
        for(var i = 0; i < 4; i++){
            if (legs_updated[i]) foot_pos[1] = y;
            else foot_pos[1] = 0;
            foot_pos[0] = this.legs[i].xg;
            foot_pos[2] = this.legs[i].zg;
            this.getLegPos(i+1,foot_pos,leg_pos,0);
            this.legs[i].inverseKinematics(leg_pos[0],leg_pos[1],leg_pos[2],this.joints.joint_angles);
        }
    }
}

export {Gait_Type, Special_Moves, Rotation_Dir, Foot_Trajectory, X_MECHDOG, Z_MECHDOG, Body};