/*
*  Author:     Geraldine Barreto
*  Version:    1.0
*  License:    GNU General Public License v3.0
*  
*  Description:  Control interface for mechDOG robot using processing & controlP5
*/

import {Quadruped} from './quadruped.js';
import {Foot_Trajectory} from './IK_quad.js';
import {loadRobotModel, robotAnimations, addOpacity} from './animations.js';
import {Communication, COMport} from './communication.js';

let wWidth, wHeight, footerHeight, canvasHeight, buttonHeight;
let header, leftCanvas, middleCanvas, rightCanvas, footer, button;
let logo, joystick1, joystick2, roboto;
let infoButton, COMmenu, BAUDmenu, emergencyButton, teachButton, haltButton;
let COMlabel, BAUDlabel, limpButton, caliButton, labname, labcolor;
let labelCMD, directCMD, sendCMD, resetB, jogB, CCW, CW, leftB, rightB, W, A, stopRobot, S, D, jy, jY, jx, jX;
let DLabel, gaitTypesw, SLabel, speedLabel, speedSel, CLabel, gaitShapesw, SQLabel;
let checkbox, checkbox2, movesLabel, movesSel, ledLabel, LEDsel, seqButton, fkButton, fsButton;
let robotJoint = [[],[],[],[]], ctrlIK = [[],[]];
let headerHeight, leftWidth, rightWidth, middleWidth;
let speed = 4, servoI = 1, servoJ = 1;
let value, buttonPos, joystickSize;
let fbrl = [0, 0, 0, 0];
comm = new Communication(COMport.OFF, 38400);
var robot = new Quadruped();
var comm, keyFlag = true, opacity = false;
var mobile = false;

class headerButtons{
  constructor(name,x,message){
    this.name = name;
    this.xPos = x;
    this.msg = message;
    this.clickEvent = this.clickEvent.bind(this);
    this.createButtons();
  }
  createButtons(){
    this.button = createButton(this.name);
    this.button.position(this.xPos,0.605*headerHeight);
    this.button.mousePressed(this.clickEvent);
    this.button.value(0);
  }
  updatePos(x) {
    this.xPos = x;
    this.button.position(x,0.605*headerHeight);
  }
  updateName(newname){
    this.button.html(newname);
  }
  clickEvent(){
    if (this.name == 'TEACH' || this.name == 'CALIBRATE' || this.name == 'LIMP'){
      if (this.button.value() == 1) this.button.value(0);
      else this.button.value(1);
    }
    else this.button.value(1);
    if (this.button.value() == 1){
      this.button.style('background-color', 'rgb(200,90,0)');
      if(comm.selected != COMport.OFF) alert("Please wait for the robot to " + this.msg);
    }
    let time = 100;
    switch(this.name){
      case 'CALIBRATE':
        if (this.button.value() == 1) limp();
        break;
      case 'LIMP':
        if (this.button.value() == 1) limp();
        else halt();
        break;
      case 'HALT & HOLD':
        halt();
        this.button.value(0);
        time = 3000;
        break;
      case 'HOLD':
        halt();
        this.button.value(0);
        time = 3000;
        break;
      case 'TEACH':
        if (this.button.value() == 1) limp();
        else halt();
        break;
    }
    if (this.button.value() == 0) delayT(time).then(() => this.button.style('background-color', 'rgb(57, 57, 57)'));
  }
}

class anglesInput{
  constructor(id,name,x,y,width,color, min, max){
    this.id = id;
    this.name = name;
    this.xPos = x;
    this.yPos = y;
    this.width = width/4;
    this.size = width/4.5;
    this.color = color;
    this.minVal = min;
    this.maxVal = max;
    this.inputEvent = this.inputEvent.bind(this);
    this.sliderEvent = this.sliderEvent.bind(this);
    this.createInputs();
  }
  createInputs() {
    this.label = createDiv(this.name);
    this.label.style('color', 'white');
    this.label.position(this.xPos,this.yPos);
    
    this.input = createInput(this.minVal+(this.maxVal-this.minVal)/2);
    this.input.style('color', this.color);
    this.input.position(this.xPos,this.yPos+this.label.height+5);
    this.input.size(this.size);
    this.input.input(this.inputEvent);

    this.slider = createSlider(this.minVal, this.maxVal, this.minVal+(this.maxVal-this.minVal)/2);
    this.slider.position(this.xPos-8,this.yPos+this.label.height+this.input.height+15);
    this.slider.style('width', (this.width).toString()+'px');
    this.slider.style('background-color', this.color);
    this.slider.changed(this.sliderEvent);
  }
  updateInputs(x,y,width) {
    this.label.show();
    this.label.position(x,y);

    this.input.show();
    this.input.position(x,y+this.label.height+5);
    this.input.size(width/4.5);

    this.slider.show();
    this.slider.position(x-6,y+this.label.height+this.input.height+15);
    this.slider.style('width', (width/4).toString()+'px');
  }
  hideInputs(){
    this.label.hide();
    this.input.hide();
    this.slider.hide();
  }
  inputEvent() {
    if(this.input.value()<this.minVal || this.input.value()>this.maxVal){
      console.log(this.input.value() + " degrees is out of the range [" + str(this.minVal) + "," + str(this.maxVal) + "]");
    }
    this.slider.value(this.input.value());
    //Forward kinematics
    if(this.id >= 11 && comm.selected != COMport.OFF){
      let angle = this.slider.value();

      //Check if direction is inverted
      if (this.id - int(this.id/10)*10 > 1) angle = -angle;
      comm.send("#" + this.id + "D" + str(angle*10) + "\r");
    }
    //Inverse kinematics
    if (this.id < 11){
      //Update animations
      switch(this.name){
        case 'ROLL':
          robot.roll(this.slider.value());
          break;
        case 'PITCH':
          robot.pitch(this.slider.value());
          break;
        case 'YAW':
          robot.yaw(this.slider.value());
          break;
        case 'X':
          robot.frontalOffset(this.slider.value());
          break;
        case 'Y':
          robot.height(this.slider.value());
          break;
        case 'Z':
          robot.lateralOffset(this.slider.value());
          break;
      }
    }
  }
  sliderEvent(){
    this.input.value(this.slider.value());
    //Forward kinematics
    if(this.id >= 11 && comm.selected != COMport.OFF){
      let angle = this.slider.value();

      //Check if direction is inverted
      if (this.id - int(this.id/10)*10 > 1) angle = -angle;
      comm.send("#" + this.id + "D" + str(angle*10) + "\r");
    }
    //Inverse kinematics
    if (this.id < 11){
      //Update animations
      switch(this.name){
        case 'ROLL':
          robot.roll(this.slider.value());
          break;
        case 'PITCH':
          robot.pitch(this.slider.value());
          break;
        case 'YAW':
          robot.yaw(this.slider.value());
          break;
        case 'X':
          robot.frontalOffset(this.slider.value());
          break;
        case 'Y':
          robot.height(this.slider.value());
          break;
        case 'Z':
          robot.lateralOffset(this.slider.value());
          break;
      }
    }
  }
}

function delayT(time) {
  return new Promise(resolve => setTimeout(resolve, time));
}

function setup(){
  //Switch width and height if the orientation changes
  if(deviceOrientation == LANDSCAPE){
    createCanvas(windowWidth,windowHeight, WEBGL);
    wWidth = windowWidth;
    wHeight = windowHeight;
  }
  else{
    createCanvas(windowHeight,windowWidth, WEBGL);
    wWidth = windowHeight;
    wHeight = windowWidth;
  }

  headerHeight = 0.09*wHeight;
  footerHeight = 1;
  
  fkButton = createButton("");
  fkButton.size(20,20);
  fkButton.html('<');
  fkButton.style('border-radius','50%');
  fkButton.style('box-shadow', 'none');
  fkButton.style('font-size','15px');
  fkButton.mousePressed(hideFK);
  fkButton.value(0);

  fsButton = createButton("");
  fsButton.size(20,20);
  fsButton.html('>');
  fsButton.style('border-radius','50%');
  fsButton.style('box-shadow', 'none');
  fsButton.style('font-size','15px');
  fsButton.style('transform','rotate(-90deg)');
  fsButton.mousePressed(fullscreen);
  fsButton.value(0);

  canvasSize();

  logo = loadImage('assets/logo.png');
  joystick1 = loadImage("assets/joystick1.png");
  joystick2 = loadImage("assets/joystick2.png");
  roboto = loadFont('assets/roboto.ttf');

  //Set up header buttons and menus
  infoButton = createSelect();
  infoButton.value(0);
  infoButton.size(18,18);
  infoButton.style('border-radius','50%');
  infoButton.style('box-shadow', '1px 1px 1px 1px black');
  infoButton.style('background-image', 'linear-gradient(to bottom right, black, grey)');
  infoButton.option('Wiki',0);
  infoButton.option('Github',1);
  infoButton.option('Community',2);
  infoButton.changed(informationB);
  //COM port menu
  COMmenu = createSelect();
  COMmenu.option('OFF', 0);
  COMmenu.option('USB', 1);
  COMmenu.option('WIFI', 2);
  COMmenu.selected('OFF');
  COMmenu.changed(menuCOM);
  COMlabel = createDiv('COM');
  COMlabel.style('color', 'rgb(57, 57, 57)');
  //Baudrate menu
  BAUDmenu = createSelect();
  BAUDmenu.option('9600');
  BAUDmenu.option('19200');
  BAUDmenu.option('38400');
  BAUDmenu.option('57600');
  BAUDmenu.option('115200');
  BAUDmenu.selected('38400');
  // BAUDmenu.disable();
  BAUDmenu.changed(menuBAUD);
  BAUDlabel = createDiv('BAUD');
  BAUDlabel.style('color', 'rgb(57, 57, 57)');
  //Emergency button
  emergencyButton = createButton('');
  emergencyButton.size(85,85);
  emergencyButton.style('box-shadow', 'none');
  emergencyButton.style('background-color','rgb(254, 175, 60)');
  emergencyButton.style('border-radius','50%');
  emergencyButton.addClass('emergency');
  emergencyButton.mousePressed(changeButtonE);
  //Teach button
  teachButton = new headerButtons('TEACH',emergencyButton.position().x-15-int(wWidth/25),'go limp');
  //Halt & Hold button
  haltButton = new headerButtons('HALT & HOLD',teachButton.xPos-62-int(wWidth/25),'halt and hold');
  //Limp button
  limpButton = new headerButtons('LIMP',haltButton.xPos-16-int(wWidth/25),'go limp');
  //Calibrate button
  caliButton = new headerButtons('CALIBRATE',limpButton.xPos-50-int(wWidth/25),'be still, place the robot in the calibration posture and press limp');

  //Left canvas inputs
  for(let i = 0; i <= 3; i++){
    for (let j = 0; j <= 2; j++){
      switch(j){
        case 0:
          labcolor = 'rgb(200,90,0)';
          switch(i){
            case 0:
              labname = 'RIGHT REAR';
              break;
            case 1:
              labname = 'RIGHT FRONT';
              break;
            case 2:
              labname = 'LEFT REAR';
              break;
            case 3:
              labname = 'LEFT FRONT';
              break;
          }
          min = -30;
          max = 60;
          break;
        case 1:
          labcolor = 'Orange';
          labname = '&nbsp;';
          min = -30;
          max = 100;
          break;
        case 2:
          labcolor = 'Black';
          labname = '&nbsp;';
          min = -200;
          max = -5;
          break;
      }
      robotJoint[i][j] = new anglesInput((i+1)*10+j+1,labname,leftWidth*(j*0.3+0.07),headerHeight+20+i*canvasHeight/5, leftWidth, labcolor, min, max);
    }
  }

  labelCMD = createDiv("DIRECT COMMAND");
  labelCMD.style('color', 'white');

  directCMD = createInput();
  directCMD.style('color', 'Black');
  directCMD.size(leftWidth*0.6,20);
  directCMD.mouseOver(keyLock);
  directCMD.mouseOut(keyUnlock);

  sendCMD = createButton("SEND");
  sendCMD.mousePressed(CMDsend);
  sendCMD.mouseOver(keyLock);
  sendCMD.mouseOut(keyUnlock);

  resetB = createButton("R");
  resetB.size(20,20);
  resetB.style('box-shadow', 'none');
  resetB.mousePressed(reset);

  jogB = createButton("J");
  jogB.size(20,20);
  jogB.style('box-shadow', 'none');
  jogB.value(0);
  jogB.mousePressed(jog);

  //Right canvas inputs
  for(let i = 0; i < 2; i++){
    for (let j = 0; j < 3; j++){
      switch(j){
        case 0:
          if (i == 0) {
            labname = "ROLL";
            min = robot.body.roll_limits[0];
            max = robot.body.roll_limits[1];
          } else {
            labname = "X";
            min = robot.body.cgx_limits[0];
            max = robot.body.cgx_limits[1];
          }
          labcolor = color(200, 90, 0);
          break;
        case 1:
          if (i == 0) {
            labname = "PITCH";
            min = robot.body.pitch_limits[0];
            max = robot.body.pitch_limits[1];
          } else {
            labname = "Y";
            min = robot.body.cgy_limits[0];
            max = robot.body.cgy_limits[1];
          }
          labcolor = color(254, 175, 60);
          break;
        case 2:
          if (i == 0) {
            labname = "YAW";
            min = robot.body.yaw_limits[0];
            max = robot.body.yaw_limits[1];
          } else {
            labname = "Z";
            min = robot.body.cgz_limits[0];
            max = robot.body.cgz_limits[1];
          }
          labcolor = color(57, 57, 57);
          break;
      }
      ctrlIK[i][j] = new anglesInput(i*3+j+2,labname,leftWidth+middleWidth+rightWidth*(j*0.3+0.07), headerHeight+20+i*canvasHeight/5, leftWidth, labcolor, min, max);
    }
  }

  CCW = createButton("");
  CCW.size(30,30);
  CCW.html('&#x21BA;');
  CCW.style('border-radius','50%');
  CCW.style('box-shadow', 'none');
  CCW.style('font-size','20px');
  CCW.mousePressed(rotateCCW);
  CCW.value(1);

  CW = createButton("");
  CW.size(30,30);
  CW.html('	&#x21BB;');
  CW.style('border-radius','50%');
  CW.style('box-shadow', 'none');
  CW.style('font-size','20px');
  CW.mousePressed(rotateCW);
  CW.value(1);

  leftB = createButton("");
  leftB.size(30,30);
  leftB.html('&#x2190;');
  leftB.style('border-radius','50%');
  leftB.style('box-shadow', 'none');
  leftB.style('font-size','20px');
  leftB.mousePressed(goLeft);
  leftB.value(1);

  rightB = createButton("");
  rightB.size(30,30);
  rightB.html('&#x2192;');
  rightB.style('border-radius','50%');
  rightB.style('box-shadow', 'none');
  rightB.style('font-size','20px');
  rightB.mousePressed(goRight);
  rightB.value(1);

  W = createCheckbox("");
  W.style('transform', 'scale(' + str(buttonHeight) + ')');
  W.style('opacity', '0');
  W.changed(forward);

  A = createCheckbox("");
  A.style('transform', 'scale(' + str(buttonHeight) + ')');
  A.style('opacity', '0');
  A.changed(left);

  stopRobot = createButton("x");
  stopRobot.size(20,20);
  stopRobot.style('opacity', '0');
  stopRobot.value(0);
  stopRobot.mousePressed(stopButton);

  S = createCheckbox("");
  S.style('transform', 'scale(' + str(buttonHeight) + ')');
  S.style('opacity', '0');
  S.changed(backward);

  D = createCheckbox("");
  D.style('transform', 'scale(' + str(buttonHeight) + ')');
  D.style('opacity', '0');
  D.changed(right);

  jY = createCheckbox("");
  jY.style('transform', 'scale(' + str(buttonHeight) + ')');
  jY.style('opacity', '0');
  jY.changed(Yplus);

  jx = createCheckbox("");
  jx.style('transform', 'scale(' + str(buttonHeight) + ')');
  jx.style('opacity', '0');
  jx.changed(Xminus);

  jX = createCheckbox("");
  jX.style('transform', 'scale(' + str(buttonHeight) + ')');
  jX.style('opacity', '0');
  jX.changed(Xplus);

  jy = createCheckbox("");
  jy.style('transform', 'scale(' + str(buttonHeight) + ')');
  jy.style('opacity', '0');
  jy.changed(Yminus);

  DLabel = createDiv("D");
  DLabel.style('color', 'white');

  gaitTypesw = createElement(
    'label',
    '<input id="toggle" type="checkbox"/><span class="slider round"></span>'); 
  gaitTypesw.addClass('switch');
  checkbox = select('#toggle');
  checkbox.changed(gaitType);

  SLabel = createDiv("S");
  SLabel.style('color', 'white');

  speedLabel = createDiv("SPEED");
  speedLabel.style('color', 'white');

  speedSel = createSelect();
  speedSel.option('1');
  speedSel.option('2');
  speedSel.option('3');
  speedSel.option('4');
  speedSel.selected('4');
  speedSel.changed(selectSpeed);
  CLabel = createDiv("C");
  CLabel.style('color', 'white');

  gaitShapesw = createElement(
    'label',
    '<input id="toggle" type="checkbox" /><span class="slider round"></span>'); 
  gaitShapesw.addClass('switch');
  checkbox2 = select('#toggle', gaitShapesw);
  checkbox2.changed(gaitShape);
  
  SQLabel = createDiv("S");
  SQLabel.style('color', 'white');

  movesSel = createSelect();
  movesSel.option('UP',0);
  movesSel.option('SIT',1);
  movesSel.option('LAY',2);
  movesSel.option('PAW',3);
  movesSel.option('WIGGLE',4);
  movesSel.option('TINKLE',5);
  movesSel.option('STRETCH',6);
  movesSel.selected('UP');
  movesSel.changed(selectMoves);
  
  if (wWidth >= 1050) movesLabel = createDiv("SPECIAL MOVES");
  else movesLabel = createDiv("MOVES");
  movesLabel.style('color', 'white');

  LEDsel = createSelect();
  LEDsel.option('OFF',0);
  LEDsel.option('RED',1);
  LEDsel.option('GREEN',2);
  LEDsel.option('BLUE',3);
  LEDsel.option('YELLOW',4);
  LEDsel.option('CYAN',5);
  LEDsel.option('PINK',6);
  LEDsel.option('WHITE',7);
  LEDsel.selected('GREEN');
  LEDsel.changed(selectLED);

  ledLabel = createDiv("LED");
  ledLabel.style('color', 'white');

  seqButton = createButton("");
  seqButton.size(20,20);
  seqButton.html('>');
  seqButton.style('border-radius','50%');
  seqButton.style('box-shadow', 'none');
  seqButton.style('font-size','15px');
  seqButton.style('transform','rotate(-90deg)');
  seqButton.mousePressed(hideSeq);
  seqButton.value(0);
  windowResized();

  //if (mobile) 
  seqButton.hide();

  //Robot Model
  if (!mobile) {
    loadRobotModel();
    robotAnimations();
  }

  reset();
}

//Direct command
function CMDsend(){
  var cmd = directCMD.value().toUpperCase();
  if (cmd.length > 0){
    sendCMD.style('background-color', 'rgb(200,90,0)');
    if (cmd.charAt(0) != '#') cmd = '#' + cmd + '\r';
    comm.send(cmd);
    delayT(20).then(() => sendCMD.style('background-color', 'rgb(57, 57, 57)'));
  }
  directCMD.value("");
}

function keyLock(){
  keyFlag = false;
}

function keyUnlock(){
  keyFlag = true;
}

function hideSeq(){
  if (seqButton.value() == 1){
    seqButton.html('>');
    seqButton.value(0);
  }
  else{
    seqButton.html('<');
    seqButton.value(1);
  }
  windowResized();
}

function hideFK(){
  if (fkButton.value() == 1){
    fkButton.html('<');
    fkButton.value(0);
  }
  else{
    fkButton.html('>');
    fkButton.value(1);
  }
  canvasSize();
  windowResized();
}

function fullscreen(){
  if (fsButton.value() == 1){
    fsButton.html('>');
    fsButton.value(0);
    document.exitFullscreen();
  }
  else{
    fsButton.html('<');
    fsButton.value(1);
    document.body.requestFullscreen();
    screen.orientation.lock("landscape");
  }
  windowResized();
}

function reset(){
  resetB.style('background-color', 'rgb(200,90,0)');
  console.log("Reset");
  for(let i = 0; i < 2; i++){
    for (let j = 0; j < 3; j++){
      ctrlIK[i][j].slider.value(0);
      ctrlIK[i][j].input.value(0);
    }
  }
  robot.roll(0);
  ctrlIK[1][1].slider.value(ctrlIK[1][1].maxVal);
  ctrlIK[1][1].input.value(ctrlIK[1][1].slider.value());
  delayT(50).then(() => resetB.style('background-color', 'rgb(57, 57, 57)'));
  delayT(200).then(() => robot.frontalOffset(0));
  delayT(400).then(() => robot.pitch(0));
  delayT(600).then(() => robot.height(ctrlIK[1][1].maxVal));
  delayT(800).then(() => robot.yaw(0));
  delayT(1000).then(() => robot.lateralOffset(0));
}

function jog(){
  if (jogB.value() == 0){
    jogB.value(1);
    jogB.style('background-color', 'rgb(200,90,0)');
    console.log("Jog On");
    robot.specialMove(7);
  }
  else if (jogB.value() == 1){
    jogB.value(0);
    jogB.style('background-color', 'rgb(57, 57, 57)');
    console.log("Jog Off");
    robot.specialMove(8); 
  }
}

function forward(){
  if (W.checked()){
    console.log("Forward");
    fbrl[0] = 1;
  }
  else{
    console.log("Stop");
    fbrl[0] = 0;
  }
}

function left(){
  if (A.checked()){
    console.log('Strafe left');
    fbrl[3] = 1;
  }
  else{
    console.log("Stop");
    fbrl[3] = 0;
  }
}

function stopButton(){
  console.log("Stop");
  fbrl = [0,0,0,0];
  W.checked(false);
  A.checked(false);
  S.checked(false);
  D.checked(false);
  CW.value(0);
  CCW.value(0);
  rotateCW();
  rotateCCW();
  if (comm.selected == COMport.WIFI) comm.send("#100M0V0\r");
}

function backward(){
  if (S.checked()){
    console.log('Backward');
    fbrl[1] = -1;
  }
  else{
    console.log("Stop");
    fbrl[1] = 0;
  }
}

function right(){
  if (D.checked()){
    console.log('Strafe right')
    fbrl[2] = -1;
  }
  else{
    console.log("Stop");
    fbrl[2] = 0;
  }
}


function rotateCCW(){
  if (CCW.value() == 1){
    console.log("Rotate CCW");
    CCW.style('background-color', 'rgb(200,90,0)');
    CCW.value(0);
    value = -1;
  }
  else{
    console.log("Stop rotation");
    CCW.style('background-color', 'rgb(57,57,57)');
    CCW.value(1);
    value = 0;
  }
  robot.rotate(value);
}

function rotateCW(){
  if (CW.value() == 1){
    console.log("Rotate CW");
    CW.style('background-color', 'rgb(200,90,0)');
    CW.value(0);
    value = 1;
  }
  else{
    console.log("Stop rotation");
    CW.style('background-color', 'rgb(57,57,57)');
    CW.value(1);
    value = 0;
  }
  robot.rotate(value);
}

function Xplus(){
  console.log('X+');
  updateSliders(1, 1, 0);
  robot.frontalOffset( ctrlIK[1][0].slider.value());
}

function Xminus(){
  console.log('X-');
  updateSliders(-1, 1, 0);
  robot.frontalOffset(ctrlIK[1][0].slider.value());
}

function Yplus(){
  console.log('Y+');
  updateSliders(1, 1, 1);
  robot.height(ctrlIK[1][1].slider.value());
}

function Yminus(){
  console.log('Y-');
  updateSliders(-1, 1, 1);
  robot.height(ctrlIK[1][1].slider.value());
}

function goLeft(){
  console.log('Z-');
  leftB.style('background-color', 'rgb(200,90,0)');
  updateSliders(-1, 1, 2);
  robot.lateralOffset(ctrlIK[1][2].slider.value());
  delayT(20).then(() => leftB.style('background-color', 'rgb(57, 57, 57)'));
}

function goRight(){
  console.log('Z+');
  rightB.style('background-color', 'rgb(200,90,0)');
  updateSliders(1, 1, 2);
  robot.lateralOffset(ctrlIK[1][2].slider.value());
  delayT(20).then(() => rightB.style('background-color', 'rgb(57, 57, 57)'));
}

function updateSliders(sign, i, j){
  if (sign > 0){
    if (int(ctrlIK[i][j].input.value())+5 <= ctrlIK[i][j].maxVal){
      ctrlIK[i][j].slider.value(int(ctrlIK[i][j].input.value())+5);
      ctrlIK[i][j].input.value(ctrlIK[i][j].slider.value());
    }
    else{
      ctrlIK[i][j].slider.value(int(ctrlIK[i][j].maxVal));
      ctrlIK[i][j].input.value(ctrlIK[i][j].slider.value());
    }
  }
  else{
    if (int(ctrlIK[i][j].input.value())-5 >= ctrlIK[i][j].minVal){
      ctrlIK[i][j].slider.value(int(ctrlIK[i][j].input.value())-5);
      ctrlIK[i][j].input.value(ctrlIK[i][j].slider.value());
    }
    else{
      ctrlIK[i][j].slider.value(int(ctrlIK[i][j].minVal));
      ctrlIK[i][j].input.value(ctrlIK[i][j].slider.value());
    }
  }
}

//Walking speed selection
function selectSpeed(){
  speed = speedSel.value();

  if (speed == 4) checkbox.checked(0);
  else checkbox.checked(1);
  
  robot.setSpeed(parseInt(speed));
  robot.changeSpeed(parseInt(speed));

  if (comm.selected == COMport.WIFI) comm.send("#100M0V" + str(robot.orientation(fbrl)) + "S" + str(speed) + "\r");
}

//Special moves
function selectMoves(){
  robot.specialMove(parseInt(movesSel.value()));
}

function selectLED(){
  comm.send("#254LED" + LEDsel.value() + "\r");
}

//Emergency Stop
function changeButtonE() {
  if (comm.selected != COMport.OFF){
    emergencyButton.style('background-color', 'rgb(200,90,0)');
    comm.send("#254RESET\r");
    delayT(500).then(() => {
      alert("Emergency stop - the robot will reset");
      halt();
    })
    delayT(3000).then(() => {
      emergencyButton.style('background-color', 'rgb(254, 175, 60)');
      comm.send("#254LED2\r");
    })
  }
}

function calibrate(){
  //TO DO: Test function
  if(comm.selected != COMport.OFF){
        comm.send("#254CO\r");
        if (comm.selected == COMport.USB){
          for(let i = 1; i <= 4; i++){
            for (let j = 1; j <= 3; j++){
              comm.send("#" + str(i) + str(j) + "QD" + "\r");
              data = comm.read("*");
              while (data == null){
                data = comm.read("*");
                if(data == "") break;
              }
              posFeedback = comm.read("\r");
              while (posFeedback == null){
                posFeedback = comm.read("\r");
                if(posFeedback == "") break;
              }
              position = split(posFeedback,"QD");
              if (position.length > 1){
                servonumber = int(position[0].substring(1));
                servoposition = int(position[1])/10;
                if (abs(servopos) > 1) {
                  alert("Calibration error in servo " + str(servonumber));
                } else {
                  alert("The robot has been calibrated");
                }
              }
            }
          }
        }
        else{
          alert("The robot has been calibrated");
        }
    }
    else{
      COMmenu.value(0);
      caliButton.button.value(0);
      caliButton.button.style('background-color', 'rgb(57, 57, 57)');
    }
}

function limp(){
  comm.send("#254L\r");
}

function halt(){
  comm.send("#254H\r");
}

function menuBAUD() {
  //Update baudrate
  comm.baud = int(BAUDmenu.value());
  //Send command
  comm.send("#254CB" + str(comm.baud) + "\r");
  delayT(1000).then(() => comm.send("#254RESET\r"));
  comm.turnOFF("Please select a contorl mode");
}

function informationB(){
  switch(parseInt(infoButton.value())){
    case 0:
      window.open('https://community.robotshop.com/blog/show/mechdog-for-beginners-advancecd-users-developers');
      break;
    case 1:
      window.open('https://github.com/Lynxmotion/mechDOG');
      break;
    case 2:
      window.open('https://www.robotshop.com/community/forum/latest');
      break;
  }
}

function menuCOM(){
  switch(parseInt(COMmenu.value())){
    case 1:
      comm.selected = COMport.USB;
      comm.initUSB();
      break;
    case 2:
      comm.selected = COMport.WIFI;
      comm.initWIFI();
      break;
    default:
      comm.selected = COMport.OFF;
      comm.turnOFF("COM port OFF");
      break;
  }
}

function getFeedback(){
  if (serial.length != 0 && comm.selected == COMport.USB){
    comm.send("#" + str(servoI) + str(servoJ) + "QD" + "\r");
    posFeedback = comm.read("\r");
    if (posFeedback){
      position = split(posFeedback,"QD");
      servonumber = int(position[0].substring(1));
      servoposition = int(position[1])/10;
      i = int(servonumber/10);
      j = servonumber - i*10;
      if (j <= 1){
        robotJoint[i-1][j-1].input.value(int(servoposition));
        robotJoint[i-1][j-1].slider.value(int(servoposition));
      }
      else{
        robotJoint[i-1][j-1].input.value(-int(servoposition));
        robotJoint[i-1][j-1].slider.value(-int(servoposition));
      }
      if (servoJ == 3) {
        servoJ = 1;
        if (servoI == 4) servoI = 1; 
        else servoI++;
      } else servoJ++;
    }
  }
  else{
    alert("Please use USB mode to get feedback");
    teachButton.button.value(0);
    teachButton.button.style('background-color', 'rgb(57, 57, 57)');
  }
}

function gaitType() {
  if (this.checked()) {
    console.log("Static");
    speedSel.value(3);
    robot.setSpeed(3);
    robot.changeSpeed(3);
  } else {
    console.log("Dynamic");
    speedSel.value(4);
    robot.setSpeed(4);
    robot.changeSpeed(4);
  }
  speed = speedSel.value();
  if(comm.selected == COMport.USB){
    switch (speed) {
      case 3:
          comm.send("#254FPC3\r");
          break;
      case 4:
          comm.send("#254FPC3\r");
          break;
    }
  }
  if (comm.selected == COMport.WIFI && speed > 0) comm.send("#100M0V" + str(robot.orientation(fbrl)) + "S" + speed + "\r");
}

function gaitShape() {
  if (this.checked()) {
    console.log("Square");
    value = 0;
    robot.gaitType(Foot_Trajectory.Square);
  } else {
    console.log("Circular");
    value = 1;
    robot.gaitType(Foot_Trajectory.Circular);
  }
}

function canvasSize(){

  canvasHeight = wHeight - headerHeight - footerHeight;
  
  buttonHeight = (wWidth*0.002+wHeight*0.004)/2;

  if (wWidth < 800 || wHeight < 500) mobile = true;
  else mobile = false;

  //Establish min size for canvases
  if (mobile){
    if (fkButton.value() == 0){
      leftWidth = 0.5*wWidth-1;
      rightWidth = 0.5*wWidth;
    }
    else{
      leftWidth = 1;
      rightWidth = wWidth-2;
    }
  }
  else{
    if (wWidth >= 1000){
      leftWidth = 0.2*wWidth;
      rightWidth = 0.2*wWidth;  
    }
    else{
      leftWidth = 200;
      rightWidth = 200;
    } 
  }
  middleWidth = wWidth - leftWidth - rightWidth;
  
  //Set up the canvases
  header = createGraphics(wWidth,headerHeight);
  leftCanvas = createGraphics(leftWidth,canvasHeight);
  middleCanvas = createGraphics(middleWidth,canvasHeight);
  rightCanvas = createGraphics(rightWidth,canvasHeight);
  footer = createGraphics(wWidth,footerHeight);
  button = createGraphics(rightWidth/12,rightWidth/12);
}

function windowResized() {
  if(windowWidth >= windowHeight){
    resizeCanvas(windowWidth,windowHeight);
    wWidth = windowWidth;
    wHeight = windowHeight;
  }
  else{
    resizeCanvas(windowHeight,windowWidth);
    wWidth = windowHeight;
    wHeight = windowWidth;
  }

  headerHeight = 0.1*wHeight;

  if (seqButton.value() == 0) footerHeight = 0;
  else{
    if (wHeight >= 700) footerHeight = 0.25*wHeight;
    else footerHeight = 150;
  }
  canvasSize();

  infoButton.position(wWidth-23, 0.07*headerHeight);

  COMmenu.position(wWidth-25-int(wWidth/25), 0.6*headerHeight);
  COMlabel.position(wWidth-COMmenu.width-45-int(wWidth/25), 0.605*headerHeight);
  BAUDmenu.position(COMlabel.position().x-BAUDmenu.width-15-int(wWidth/25),0.6*headerHeight);
  BAUDmenu.show();
  BAUDlabel.position(BAUDmenu.position().x-50, 0.605*headerHeight);
  emergencyButton.position(BAUDlabel.position().x-40-int(wWidth/25), 0.5*headerHeight);
  emergencyButton.size(85,85);
  teachButton.updatePos(emergencyButton.position().x-15-int(wWidth/25));
  if (wWidth >= 1000){
    movesLabel.html('SPECIAL MOVES');
    haltButton.updateName('HALT & HOLD');
    COMlabel.html('COM');
    BAUDlabel.html('BAUD');
    haltButton.updatePos(teachButton.xPos-62-int(wWidth/25));
  }
  if (wWidth < 1000){
    movesLabel.html('MOVES');
    haltButton.updateName('HOLD');
    COMlabel.html('COM');
    BAUDlabel.html('BAUD');
    COMlabel.position(wWidth-COMmenu.width-40-int(wWidth/25), 0.605*headerHeight);
    BAUDlabel.position(BAUDmenu.position().x-43, 0.605*headerHeight);
    emergencyButton.position(BAUDlabel.position().x-40-int(wWidth/25), 0.5*headerHeight);
    if(mobile){
      COMlabel.html('');
      BAUDmenu.position(COMlabel.position().x-3-int(wWidth/25),0.6*headerHeight);
      BAUDmenu.hide();
      BAUDlabel.html('');
      emergencyButton.position(COMmenu.position().x-45-int(wWidth/25), 0.35*headerHeight);
      teachButton.updatePos(emergencyButton.position().x-25-int(wWidth/25));
      emergencyButton.size(70,70);
    }
    haltButton.updatePos(teachButton.xPos-17-int(wWidth/25));
  }
  limpButton.updatePos(haltButton.xPos-16-int(wWidth/25));
  caliButton.updatePos(limpButton.xPos-50-int(wWidth/25));

  if (fkButton.value() == 0) fkButton.position(leftWidth-25,headerHeight+10);
  else fkButton.position(15,headerHeight+10);
  if (!mobile) fkButton.hide();
  else fkButton.show();

  //Left canvas inputs
  for(let i = 0; i <= 3; i++){
    for (let j = 0; j <= 2; j++){
      if (fkButton.value() == 0) robotJoint[i][j].updateInputs(leftWidth*(j*0.3+0.07),headerHeight+20+i*canvasHeight/5,leftWidth);
      else robotJoint[i][j].hideInputs();
    }
  }
  if (fkButton.value() == 0) {
    labelCMD.show();
    directCMD.show();
    sendCMD.show();
    labelCMD.position(leftWidth*0.06,headerHeight+25+canvasHeight*4/5);
    directCMD.position(leftWidth*0.06,headerHeight+25+canvasHeight*4.3/5);
    directCMD.size(leftWidth*0.6,20);
    sendCMD.position(leftWidth*0.72,headerHeight+27+canvasHeight*4.3/5);
  }
  else{
    labelCMD.hide();
    directCMD.hide();
    sendCMD.hide();
  }

  //Right canvas inputs
  for(let i = 0; i < 2; i++){
    for (let j = 0; j < 3; j++){
      ctrlIK[i][j].updateInputs(leftWidth+middleWidth+rightWidth*(j*0.3+0.07), headerHeight+20+i*canvasHeight/5, rightWidth);
    }
  }
  let buttonsPos;
  if(mobile) buttonsPos = headerHeight+canvasHeight/2.2;
  else buttonsPos = headerHeight+canvasHeight/2.6;

  CCW.position(leftWidth+middleWidth+rightWidth/11-1,buttonsPos);
  CW.position(leftWidth+middleWidth+rightWidth/2-42,buttonsPos);
  leftB.position(leftWidth+middleWidth+rightWidth/2+20,buttonsPos);
  rightB.position(leftWidth+middleWidth+rightWidth-47,buttonsPos);

  joystickSize = (canvasHeight+leftWidth+rightWidth)/10;
  buttonPos = -1/2*wHeight+headerHeight+canvasHeight/2.2+joystickSize/6.5;

  W.position(leftWidth+middleWidth+rightWidth/4-5,buttonPos+1/2*wHeight);
  W.style('transform', 'scale(' + str(buttonHeight) + ')');
  A.position(leftWidth+middleWidth+rightWidth/4-(canvasHeight+leftWidth+rightWidth)/35,buttonPos+joystickSize*0.25+1/2*wHeight);
  A.style('transform', 'scale(' + str(buttonHeight) + ')');
  stopRobot.position(leftWidth+middleWidth+rightWidth/4,buttonPos+joystickSize*0.25+1/2*wHeight);
  S.position(leftWidth+middleWidth+rightWidth/4-5,buttonPos+joystickSize*0.53+1/2*wHeight);
  S.style('transform', 'scale(' + str(buttonHeight) + ')');
  D.position(leftWidth+middleWidth+rightWidth/4+(canvasHeight+leftWidth+rightWidth)/40,buttonPos+joystickSize*0.25+1/2*wHeight);
  D.style('transform', 'scale(' + str(buttonHeight) + ')');

  jY.position(leftWidth+middleWidth+rightWidth*3/4-10,buttonPos+1/2*wHeight);
  jY.style('transform', 'scale(' + str(buttonHeight) + ')');
  jx.position(leftWidth+middleWidth+rightWidth*3/4-(canvasHeight+leftWidth+rightWidth)/30,buttonPos+joystickSize*0.25+1/2*wHeight);
  jx.style('transform', 'scale(' + str(buttonHeight) + ')');
  jy.position(leftWidth+middleWidth+rightWidth*3/4-10,buttonPos+joystickSize*0.53+1/2*wHeight);
  jy.style('transform', 'scale(' + str(buttonHeight) + ')');
  jX.position(leftWidth+middleWidth+rightWidth*3/4+(canvasHeight+leftWidth+rightWidth)/60,buttonPos+joystickSize*0.25+1/2*wHeight);
  jX.style('transform', 'scale(' + str(buttonHeight) + ')');

  DLabel.position(leftWidth+middleWidth+rightWidth/8,headerHeight+canvasHeight*4/5);
  gaitTypesw.position(leftWidth+middleWidth+rightWidth/4-17,headerHeight+canvasHeight*4/5);
  SLabel.position(leftWidth+middleWidth+rightWidth/8+rightWidth/4,headerHeight+canvasHeight*4/5);

  resetB.position(leftWidth+middleWidth+rightWidth/2+15,headerHeight+canvasHeight*3.5/5);
  jogB.position(leftWidth+middleWidth+rightWidth/2-25,headerHeight+canvasHeight*3.5/5);

  speedLabel.position(leftWidth+middleWidth+rightWidth/2-15,headerHeight+canvasHeight*3.8/5);
  speedSel.position(leftWidth+middleWidth+rightWidth/2-10,speedLabel.position().y+20);
  if (mobile) speedLabel.style('font-size', '13px');
  else speedLabel.style('font-size', '15px');

  CLabel.position(leftWidth+middleWidth+rightWidth/8+rightWidth/2,headerHeight+canvasHeight*4/5);
  gaitShapesw.position(leftWidth+middleWidth+rightWidth*3/4-17,headerHeight+canvasHeight*4/5);
  SQLabel.position(leftWidth+middleWidth+rightWidth-rightWidth/8,headerHeight+canvasHeight*4/5);

  movesSel.position(leftWidth+middleWidth+rightWidth*0.07,headerHeight+25+canvasHeight*4.3/5);
  movesLabel.position(leftWidth+middleWidth+rightWidth*0.07,movesSel.position().y-20);
  if (mobile) movesLabel.style('font-size', '13px');
  else movesLabel.style('font-size', '15px');

  LEDsel.position(leftWidth+middleWidth+rightWidth*3/4-20,headerHeight+25+canvasHeight*4.3/5);
  ledLabel.position(leftWidth+middleWidth+rightWidth*3/4-20,LEDsel.position().y-20);
  if (mobile) ledLabel.style('font-size', '13px');
  else ledLabel.style('font-size', '15px');
  
  fsButton.position(leftWidth+middleWidth+rightWidth-30,headerHeight+10);
  seqButton.position(leftWidth+middleWidth+rightWidth-30,wHeight-footerHeight-30);
  if (mobile) seqButton.hide();
  else seqButton.show();

  // //Footer inputs
  // for(let i = 1; i < 6; i++){
  //   triggerMENU[i].position(leftWidth*0.07,headerHeight+canvasHeight+0.15*i*footerHeight);
  //   sequence[i].position(leftWidth*0.07+90,headerHeight+canvasHeight+0.15*i*footerHeight);
  //   for(let j = 0; j <= frameN; j++){
  //     frameIN[i][j].position(leftWidth*0.07+190+110*j,headerHeight+canvasHeight+0.15*i*footerHeight);
  //     frameMOD[i][j].position(leftWidth*0.07+260+110*j,headerHeight+canvasHeight+0.15*i*footerHeight);
  //   }
  // }
}

function keyPressed() {
  if (keyFlag) {
    switch (key) {
      case 'w':
      case 'W':
      case 'ArrowUp':
        W.checked(true);
        forward();
        break;
      case 'a':
      case 'A':
        A.checked(true);
        left();
        break;
      case 'd':
      case 'D':
        D.checked(true);
        right();
        break;
      case 's':
      case 'S':
      case 'ArrowDown':
        S.checked(true);
        backward();
        break;   
      case 'e':
      case 'E':
      case 'ArrowRight':
        CW.value(1);
        rotateCW();
        break;
      case 'q':
      case 'Q':
      case 'ArrowLeft':
        CCW.value(1);
        rotateCCW();
        break;
      case 'r':
      case 'R':
        Yplus();
        robot.height(ctrlIK[1][1].slider.value());
        break;
      case 'f':
      case 'F':
        Yminus();
        robot.height(ctrlIK[1][1].slider.value());
        break;
      case 't':
      case 'T':
        console.log('Roll-')
        updateSliders(-1,0,0);
        robot.roll(ctrlIK[0][0].slider.value());
        break;
      case 'g':
      case 'G':
        console.log('Roll+')
        updateSliders(1,0,0);
        robot.roll(ctrlIK[0][0].slider.value());
        break;
      case 'y':
      case 'Y':
        console.log('Pitch-')
        updateSliders(-1,0,1);
        robot.pitch(ctrlIK[0][1].slider.value());
        break;
      case 'h':
      case 'H':
        console.log('Pitch+')
        updateSliders(1,0,1);
        robot.pitch(ctrlIK[0][1].slider.value());
        break;
      case 'u':
      case 'U':
        console.log('Yaw-')
        updateSliders(-1,0,2);
        robot.yaw(ctrlIK[0][2].slider.value());
        break;
      case 'j':
      case 'J':
        console.log('Yaw+')
        updateSliders(1,0,2);
        robot.yaw(ctrlIK[0][2].slider.value());
        break;
      case 'x':
      case 'X':
        movesSel.value('UP');
        key.value = 10;
        selectMoves();
        break;
      case 'z':
      case 'Z':
        movesSel.value('SIT');
        key.value = 11;
        selectMoves();
        break;
      case 'l':
      case 'L':
        movesSel.value('LAY');
        key.value = 12;
        selectMoves();
        break;
      case 'v':
      case 'V':
        movesSel.value('PAW');
        key.value = 13;
        selectMoves();
        break;
      case 'c':
      case 'C':
        movesSel.value('WIGGLE');
        key.value = 14;
        selectMoves();
        break;
      case 'b':
      case 'B':
        movesSel.value('TINKLE');
        key.value = 15;
        selectMoves();
        break;
    }
  }
};

function keyReleased(){
  if (keyFlag){
    switch (key) {
      case 'w':
      case 'W':
      case 'ArrowUp':
        W.checked(false);
        forward();
        break;
      case 'a':
      case 'A':
        A.checked(false);
        left();
        break;
      case 'd':
      case 'D':
        D.checked(false);
        right();
        break;
      case 's':
      case 'S':
      case 'ArrowDown':
        S.checked(false);
        backward();
        break;   
      case 'e':
      case 'E':
      case 'ArrowRight':
        CW.value(0);
        rotateCW();
        break;
      case 'q':
      case 'Q':
      case 'ArrowLeft':
        CCW.value(0);
        rotateCCW();
        break;
      default:
        break;
    }
  }
};

function draw(){
  drawMiddleCanvas();
  image(middleCanvas, -1/2*wWidth+leftWidth, -1/2*wHeight+headerHeight);
  drawLeftCanvas();
  drawRightCanvas();
  drawHeader();
  image(header, -1/2*wWidth, -1/2*wHeight);
  drawFooter();
}

function drawHeader(){
  noStroke();
  fill(57);
  let posHeader = headerHeight*0.5;
  rect(-1/2*wWidth, -1/2*wHeight, wWidth, posHeader);
  fill(254,175,60);
  rect(-1/2*wWidth, -1/2*wHeight+posHeader, wWidth, headerHeight-posHeader);
  image(logo, -1/2*wWidth, -1/2*wHeight+0.05*headerHeight, 3.3*headerHeight, 1.2*headerHeight);
  textFont(roboto);
  textSize(0.3*headerHeight);
  fill(255, 255, 255);
  text('mechDOG', 1/2*wWidth-1.7*headerHeight,-1/2*wHeight+0.3*headerHeight);
}

function drawLeftCanvas() {
  leftCanvas.background(125);
  image(leftCanvas, -1/2*wWidth, -1/2*wHeight+headerHeight);
}

function drawMiddleCanvas(){
  if (teachButton.button.value() == 1) getFeedback();
  if (caliButton.button.value() == 1 && limpButton.button.value() == 1){
    calibrate();
    caliButton.button.value(0);
    limpButton.button.value(0);
    delayT(3000).then(() =>{
      caliButton.button.style('background-color', 'rgb(57, 57, 57)');
      limpButton.button.style('background-color', 'rgb(57, 57, 57)');
    });
  }
  if (robot.body.new_director_angle != robot.orientation(fbrl)){
    robot.updateAngle(robot.orientation(fbrl));
    if (comm.selected == COMport.WIFI && robot.orientation(fbrl) != 0) comm.send("#100M0V" + str(robot.orientation(fbrl)) + "S" + str(speed) + "\r");
    else if (comm.selected == COMport.WIFI && robot.orientation(fbrl) == 0) comm.send("#100M0V0\r");
  }
  robot.loop();
  if ((comm.selected != COMport.USB || robot.body.stopped) && !mobile){
    robotAnimations();
    opacity = true;
  }
  else if (opacity == true){
    addOpacity();
    opacity = false;
  }
}

function drawRightCanvas() {
  rightCanvas.background(125);
  image(rightCanvas, -1/2*wWidth+leftWidth+middleWidth, -1/2*wHeight+headerHeight);
  image(joystick1, -1/2*wWidth+leftWidth+middleWidth+rightWidth/4-joystickSize/2.3, -1/2*wHeight+headerHeight+canvasHeight/2.2,joystickSize,joystickSize);
  image(joystick2, -1/2*wWidth+leftWidth+middleWidth+rightWidth*3/4-joystickSize/2.1, -1/2*wHeight+headerHeight+canvasHeight/2.2,joystickSize,joystickSize);
  button.style('width', str(int(joystickSize/6)) + 'px');
  button.style('height', str(int(joystickSize/6)) + 'px');
  if (W.checked()){
    button.background('rgba(57,57,57,0.5)');
    image(button, -1/2*wWidth+leftWidth+middleWidth+rightWidth/4,buttonPos);
    button.clear();
  }
  if (A.checked()){
    button.background('rgba(57,57,57,0.5)');
    image(button, -1/2*wWidth+leftWidth+middleWidth+rightWidth/4-joystickSize/4-5,buttonPos+joystickSize*0.25);
    button.clear();
  }
  if (S.checked()){
    button.background('rgba(57,57,57,0.5)');
    image(button, -1/2*wWidth+leftWidth+middleWidth+rightWidth/4,buttonPos+joystickSize*0.53);
    button.clear();
  }
  if (D.checked()){
    button.background('rgba(57,57,57,0.5)');
    image(button, -1/2*wWidth+leftWidth+middleWidth+rightWidth/4+joystickSize/4,buttonPos+joystickSize*0.25);
    button.clear();
  }
}

function drawFooter(){
  footer.background(57);
  image(footer, -1/2*wWidth, 1/2*wHeight-footerHeight);
}

window.setup = setup;
window.draw = draw;
window.windowResized = windowResized;
window.keyPressed = keyPressed;
window.keyReleased = keyReleased;

export {robot, robotJoint, ctrlIK, comm, CW, CCW, speed, canvasHeight, middleWidth, wWidth, wHeight};
export {teachButton, haltButton, limpButton, caliButton, COMmenu, delayT, opacity, mobile, LEDsel};