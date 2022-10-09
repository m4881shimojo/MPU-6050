/**
 * MPU6050 Roll,Pitch,Yaw rotation shimojo 2022.06.09
 *Processingの座標系は左手座標系　shimojo
 *
 * 3D Builder Unity OpenGL 等右手座標系
 * 今回モデルは3D Bulderで作った。左手座標系としたが、あえてZ文字は直さなかった。
 * Roll,Pitch,Yaw(radian) data via COM4 port
 *角度の単位に注意すること。基本はradianです。
 * 単位を変換したい場合は、degrees()やradians()を使います
 * 今回は、初期姿勢の設定には感覚的に分かりやすいdegreesを利用している
 * EX.rotateY(radians(-30.0)) <--radiansで変換している
 *
**/



PShape rocket;
import processing.serial.*;

Serial myPort;
float pitch, yaw, roll;
float rr;

// 最初の設定
void setup(){
  size(800, 600, P3D); //P3Dと書くことによって、3D空間であることを明示する
//「COM4」とは、serialが接続されているCOMポート番号をさします。
    myPort = new Serial(this, "COM4", 115200); 

  rocket = loadShape("shuttleXYZLeftAxis.obj");//space shuttle
  //rr=0.0;
   
}

// 図を描く部分
void draw(){
  background(0);
  lights();
  translate(width/2, height/2 , -200);    

  rotateX(radians(90.0));//degree
  rotateZ(radians(-90.0));//degree
  rotateY(radians(-30.0));//degree
  
  //rotateX(radians(-roll));    //degree
  //rotateY(radians(pitch));    //degree
  //rotateZ(radians(-yaw));    //degree
  rotateX(-roll);    //raddian
  rotateY(pitch);    //radian
  rotateZ(-yaw);    //radian
  scale(2.0);//2倍に拡大
  shape(rocket);//shuttele
  //rr++;
}

// シリアルポートからデータを受け取る
void serialEvent(Serial myPort){
  String myString = myPort.readStringUntil('\n');

  if (myString != null) {
    myString = trim(myString);
  
    float sensors[] = float(split(myString, ','));
    if (sensors.length > 2) {
        roll = sensors[0];
        pitch = sensors[1];
        yaw= sensors[2];
        //gx = sensors[3];
        //gy = sensors[4];
        //gz = sensors[5];
    }  
  }
}
