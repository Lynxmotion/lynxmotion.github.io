/*
 *	Authors:		Geraldine Barreto
 *					Eduardo Nunes
 *	Version:		1.0
 *  License:        GNU General Public License v3.0
 *	
 *	Description:	Lynxmotion's mechDOG robot control
 */

import {Body, Gait_Type, Special_Moves, Rotation_Dir} from './IK_quad.js';
import {COMport} from './communication.js';
import{comm} from './canvas.js';

var StopWalk = 0;
var StopMoveSpeed = 5;
var SpecialMoveSpeed = 0;           
          
var ControlMode = {           
    NoControlSelected: 0,           
    WifiStreaming: 1,           
    WifiRC: 2,           
    USB: 3           
};           

class DTime{
    constructor(dt){
        this.new_sample = 0;
        this.old_sample = Date.now();
        this.dt = dt;
    }
    updateDT(dt){
        if(dt > 0) this.dt = dt;
    }
    getDT(debug){
        this.new_sample = Date.now();
        let dt = this.new_sample - this.old_sample;

        if (debug){
            console.log("Time Error: " + (dt - this.dt));
            console.log("Time elapsed: " + dt);
        }

        if (dt - this.dt >= 0){
            this.old_sample = this.new_sample;
            return true;
        }else{
            return false;
        }
    }
    reset(){
        this.old_sample = 0;
    }
}

class Quadruped{
    constructor() {
        this.speed = 4;
        this.move_flag = true;
        this.ctrlSelected = ControlMode.NoControlSelected;
        this.body = new Body();
        this.dt = new DTime(50);
        this.changeSpeed(this.speed);
    }
    updateAngle(angle){
        this.move_flag = true;
        this.body.update_flag = true;
        if(angle == 0){
            this.body.new_move_state = 0;
            if(this.body.new_jog_mode) this.body.new_move_state = 1;
        }else{
            this.body.new_move_state = 1;
        }
        this.body.new_director_angle = angle;

        if(this.body.stopped && this.body.new_move_state == StopWalk){
            this.changeSpeed(StopMoveSpeed);
        }else if(this.speed != this.actual_speed){        
            this.changeSpeed(this.speed);
        }  
    }
    rotate(dir){
        this.move_flag = true;
        this.body.update_flag = true;
        this.body.new_rot_angle = dir;
        if(!this.body.stopped && this.body.sp_move == Special_Moves.UP && this.speed != this.actual_speed && this.body.new_rot_angle != Rotation_Dir.StopRotation){
            this.changeSpeed(this.speed);
        }
        if (comm.selected == COMport.WIFI) comm.send("#100M1V" + str(dir) + "\r");
    }
    frontalOffset(mm) {
        this.move_flag = true;
        if(this.body.stopped && this.body.sp_move == Special_Moves.UP){
            if(mm >= this.body.cgx_limits[0] && mm <=this.body.cgx_limits[1]) this.body.cgx = mm;
            if(StopMoveSpeed!= this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M5V" + str(mm) + "\r");
    }
    lateralOffset(mm) {
        this.move_flag = true;
        if(this.body.stopped && this.body.sp_move == Special_Moves.UP){
            if(mm >= this.body.cgz_limits[0] && mm <=this.body.cgz_limits[1]) this.body.cgz = mm;
            if(StopMoveSpeed!= this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M7V" + str(int(mm)) + "\r");
    }
    height(mm) {
        this.move_flag = true;
        if(this.body.sp_move == Special_Moves.UP && mm >= this.body.cgy_limits[0] && mm <=this.body.cgy_limits[1]) this.body.cgy = mm;
        if(this.body.stopped){
            if(StopMoveSpeed!= this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M6V" + str(int(mm)) + "\r");
    }
    pitch(angle){
        this.move_flag = true;
        if(this.body.sp_move == Special_Moves.UP && angle >= this.body.pitch_limits[0] && angle <= this.body.pitch_limits[1]) this.body.pitch = radians(angle);
        if(this.body.stopped){
            if(StopMoveSpeed!= this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M3V" + str(int(angle)) + "\r");
    }
    roll(angle){
        this.move_flag = true;
        if(this.body.sp_move == Special_Moves.UP && angle >= this.body.roll_limits[0] && angle <= this.body.roll_limits[1]) this.body.roll = radians(angle);
        if(this.body.stopped){
            if(StopMoveSpeed!= this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M2V" + str(int(angle)) + "\r");
    }
    yaw(angle){
        this.move_flag = true;
        if(this.body.sp_move == Special_Moves.UP && angle >= this.body.yaw_limits[0] && angle <= this.body.yaw_limits[1]) this.body.yaw = radians(angle);
        if(this.body.stopped){
            if(StopMoveSpeed != this.actual_speed) this.changeSpeed(StopMoveSpeed);
        }
        if(comm.selected == COMport.WIFI) comm.send("#100M4V" + str(int(angle)) + "\r");
    }
    gaitMode(mode){
        this.body.beta = mode;
        this.body.changeRobotPosture();
    }
    gaitType(type){
        this.move_flag = true;
        this.body.trajectory_type = type;
        if (comm.selected == COMport.WIFI) comm.send("#100M8V" + str(type) + "\r");
    }
    setSpeed(speed){
        this.speed = speed;
    }
    changeSpeed(speed) {
        this.actual_speed = speed;
        switch (speed) {
            case StopMoveSpeed:
                this.dt.updateDT(60);
                if (this.speed == 4) this.body.new_beta = Gait_Type.Dynamic;
                else this.body.new_beta = Gait_Type.Static;
                if(comm.selected == COMport.USB) comm.send("#254FPC14\r");
                break;
            case SpecialMoveSpeed:
                this.dt.updateDT(180);
                if(comm.selected == COMport.USB) comm.send("#254FPC14\r");
                break;
            case 1:
                this.dt.updateDT(70);
                this.body.new_beta = Gait_Type.Static;
                if(comm.selected == COMport.USB) comm.send("#254FPC4\r");
                break;
            case 2:
                this.dt.updateDT(60);
                this.body.new_beta = Gait_Type.Static;
                if(comm.selected == COMport.USB) comm.send("#254FPC4\r");
                break;
            case 3:
                this.dt.updateDT(50);
                this.body.new_beta = Gait_Type.Static;
                if(comm.selected == COMport.USB) comm.send("#254FPC3\r");
                break;
            case 4:
                this.dt.updateDT(50);
                this.body.new_beta = Gait_Type.Dynamic;
                if(comm.selected == COMport.USB) comm.send("#254FPC3\r");
                break;
            default:
                break;
        }
    }
    orientation(fbrl){
        var rl = fbrl[2] + fbrl[3];
        var fb = fbrl[0] + fbrl[1];
        var angle = 0;
        if (rl != 0 && fb != 0) {
            angle = Math.PI / 4 * fb;
            if (rl == -1) {
            angle += Math.PI / 2 * fb;
            }
        } else if (fb != 0) {
            angle = Math.PI / 2 * fb;
        } else if (rl != 0) {
            angle = (1 - rl) * Math.PI / 2;
        }
        var walkAngle = int(degrees(angle) + 270);
        if (walkAngle > 360) walkAngle = walkAngle - 360;
        if (rl == 0 && fb == 0) return 0;
        return walkAngle;
    }
    loop(){
        if(this.dt.getDT()){
            if(this.move_flag || !this.body.stopped){
                if(this.body.sp_move == Special_Moves.UP){
                    this.body.walk();  //Might change robot.sp_move
                    this.body.joints.moveBody();
                    if(this.body.sp_move != Special_Moves.UP) this.changeSpeed(SpecialMoveSpeed);
                }
                if(this.body.sp_move != Special_Moves.UP){
                    this.body.specialMoves();
                    this.body.joints.moveBody();
                    if(this.body.sp_move == Special_Moves.UP) this.changeSpeed(StopMoveSpeed);
                }
                this.move_flag = false;
            }
        }
    }
    specialMove(move){
        if(comm.selected == COMport.WIFI) comm.send("#100M" + str(10 + move) + "\r");
        if(move == Special_Moves.JOG_ON){
            if(!this.body.new_jog_mode){
                this.body.new_sp_move = Special_Moves.UP;
                this.body.new_jog_mode = true; 
                this.updateAngle(0);
            }
        }
        else if(move == Special_Moves.JOG_OFF){
            if(this.body.new_jog_mode){
                this.body.new_sp_move = Special_Moves.UP;
                this.body.new_jog_mode = false; 
                this.updateAngle(0);
            }
        }
        else if(this.body.new_sp_move != move){
            this.body.new_sp_move = move;
            this.move_flag = true;
            this.body.update_flag = true;
            this.body.new_move_state = StopWalk;
            this.body.new_rot_angle = Rotation_Dir.StopRotation;
        }
    }
}

export {Quadruped, StopMoveSpeed, SpecialMoveSpeed};