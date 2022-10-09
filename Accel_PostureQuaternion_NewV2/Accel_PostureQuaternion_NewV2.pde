//ジャイロセンサの姿勢とその時の加速度センサの出力を表示する
//回転座標系での元の座標系から見てどの座標に移ったかを調べる方法は
//@masakiyamabeの方法を参考にした
//https://qiita.com/masakiyamabe/items/5790052f062e3232df8d
//class Quaternion の使い方（定義）をよく参照して利用すること
//回転軸を2通り表示する
//(R_axis=0)：回転した軸周りの回転（ex: x--> y'--> z"）
//(R_axis=1)：固定軸周りの回転（ex: X--> Y--> Z）
//動画を作るには　saveFrame("frames/####.png");のコメントアウトを外す


//PVector target;//任意のベクトル（今回は利用しなかった）
Quaternion q;//クオータニオン

PVector axisX;//旧座標（固定座標系）
PVector axisY;//旧座標（固定座標系）
PVector axisZ;//旧座標（固定座標系）

float camdst, invwidth;//カメラの距離、表示窓の逆数
float da;//Rotation increment
int R_axis;//Rotetion axis 固定軸or回転軸

void setup(){
size(800, 800, P3D);
  //camdst = height * 0.86602;
  camdst = height * 0.3;
  invwidth = 1.0 / float(width);
  
  //座標変換前のx,y,z単位ベクトル（固定座標系）
  axisX = new PVector(1,0,0);
  axisY = new PVector(0,1,0);
  axisZ = new PVector(0,0,1);
  
  //変換前の座標系の任意ベクトル（今回は利用しなかった）
  //target = new PVector(50,-50,0);//任意のベクトル
  
  //クォータニオンの初期化
  q = new Quaternion();//宣言
  q.setAngleAxis( 0, new PVector(0,0,0) );//初期化

  //noLoop();//一回だけdrawを実行

  da=0.0;//inceremental rotation
  
  /* Flag indicating rotation around a fixed axis or rotation around a rotary axis
   *0-->rotation around a rotary axis（ex: x--> y'--> z"）
   *1--> rotation around a fixed axis（ex: X--> Y--> Z）
  */
  R_axis=0; //Flag of Rotation system
}


void draw(){
  background(255,255,255);
  delay(100);//100ms delay
  
  //カメラ視点の設定
  //画面の中心に原点を持ってくる
  translate(width/2, height/2,0);  
  //可動視点；マウスを使って視線移動
/*
  //float camt = -TWO_PI + mouseX * invwidth * PI;//マウスで調整
  float camt =TWO_PI*( mouseX * invwidth -0.5);//-PI～PI
  float camty =TWO_PI*( mouseY * invwidth -0.5);//-PI～PI
  //camera(cos(camt) * camdst, camdst, sin(camt) * camdst,0.0, 0.0, 0.0,1.0, 1.0, -1.0);
  float cx,cy,cz;
  cx=camdst*sin(camt)*cos(camty); cy=camdst*sin(camt)*sin(camty); cz=camdst*cos(camt);
  camera(cx,cz,cy,0.0, 0.0, 0.0,1.0, 1.0, -1.0); 
*/
 //固定視点（processingは左手座標系-->残念）
   camera(camdst,camdst,camdst,0,0,0,0,0,-1);
  

 /* テキストを画面上に表示*/
  pushMatrix();
  //rotateX(radians(0));
  //rotateY(radians(0));
  rotateZ(radians(-45)); 
  fill(0);text( "Gyro sensor attitude and it's acceleration outputs", -100, 200, 300, 30); 
  popMatrix();
  
  // Background sphere. 球を書く
  pushMatrix();
  rotateX(radians(0));
  rotateY(radians(90));
  rotateZ(radians(90));
  noFill();
  strokeWeight(1.0);
  stroke(0x3f000000);
  sphere(100);//scalar --> radius of the sphere
  popMatrix();
  
  // XYZ axis
  float scln=150;//座標軸の長さ
  stroke(0,0,0); fill(255,0,0);//矢印の色と円錐の色
  strokeWeight(2);
  arrow(0, 0, 0, scln*axisX.x, scln*axisX.y, scln*axisX.z);//x 
  stroke(0,0,0); fill(0,255,0);//矢印の色と円錐の色
  strokeWeight(2); 
  arrow(0, 0, 0, scln*axisY.x, scln*axisY.y, scln*axisY.z);//y   
  stroke(0,0,0); fill(0,0,255);//矢印の色と円錐の色
  strokeWeight(2); 
  arrow(0, 0, 0, scln*axisZ.x, scln*axisZ.y, scln*axisZ.z);//z
  textSize(20);
  text("Z",scln*axisZ.x, scln*axisZ.y, scln*axisZ.z);
  strokeWeight(1);  
  // -- ここまで準備    
  
  
  // -- ここから本題
  float x_rot,y_rot,z_rot;
  x_rot=30.0+da; y_rot=30.0+da; z_rot=0.0+da;//da-->increment rotation
  
  //回転前に座標系を保存（通常通り）  
  pushMatrix(); 
  
  if(R_axis==0)
{
  //座標系をX軸中心に45度、Y軸中心に10度、Z軸に30度回転（通常通り）
  rotateX( radians(x_rot) ); //X軸を中心にx_rot degree
  rotateY( radians(y_rot) ); //Y軸を中心にy_rot degree
  rotateZ( radians(z_rot) ); //Z軸を中心にz_rot degree
}else{
  rotateZ( radians(z_rot) ); //Z軸を中心にz_rot degree 
  rotateY( radians(y_rot) ); //Y軸を中心にy_rot degree
  rotateX( radians(x_rot) ); //X軸を中心にx_rot degree
} 
  stroke(0,0,0); strokeWeight(3);
  fill(255,100,200);
  //noFill();
  box(60,40,10);//原点に長方形板
  
  //回転情報を上と同じように順にクォータニオンに記録していく（ここ重要。あとで使う）
  //println(q.x,q.y,q.z,q.w);//0.0 0.0 0.0 1.0-->最初の値

  q.setAngleAxis( 0, new PVector(0,0,0) );//initialize q
  if(R_axis==0)
{
  q = q.mult( new Quaternion( radians(x_rot), q.mult(axisX) ) );  //X軸を中心に回転
  q = q.mult( new Quaternion( radians(y_rot), q.mult(axisY) ) );  //Y軸を中心に回転
  q = q.mult( new Quaternion( radians(z_rot), q.mult(axisZ) ) ); //Z軸を中心に回転
}else{
  q = q.mult( new Quaternion( radians(z_rot), q.mult(axisZ) ) );  //Z軸を中心
  q = q.mult( new Quaternion( radians(y_rot), q.mult(axisY) ) );  //Y軸を中心に
  q = q.mult( new Quaternion( radians(x_rot), q.mult(axisX) ) );  //X軸を中心に
}
  
  //X,Y,Z軸回転後のquaternionが計算された
  //println(q.x,q.y,q.z,q.w);//0.3890777 -0.020891182 0.2704243 0.88037086
  
  //目的の座標に動かす（通常通り）
  //translate(target.x, target.y, target.z);  
  popMatrix();//回転前の座標系に戻す（通常通り）
  
  
  // -- ここから元の座標系における回転変換後の位置を計算していく
  //.mult　は<float r>,<Quaternion q>,<PVector v>の場合がある
  
  // 1. クォータニオンを使って回転させた座標系のX軸、Y軸、Z軸が元の座標系でどのようなベクトルになっているか調べる（全て単位ベクトル）
  PVector newAxisX = q.mult(axisX);
  PVector newAxisY = q.mult(axisY);
  PVector newAxisZ = q.mult(axisZ);
  
  // 2. 上の各単位ベクトルからの位置を計算（今回の例では利用せず）
   //Targetベクトルは任意のベクトル（ここでは任意のベクトルを回転移動する例として取り上げた。）   
  //PVector newTarget = PVector.add( PVector.mult(newAxisX,target.x), PVector.mult(newAxisY,target.y), PVector.mult(newAxisZ,target.z) );
  
  //ちなみに、1と2のステップは下記のように1ステップで求めることもできる
  //軸がどっちを向いているか知る必要がない場合はこれで事足りる。各軸の方向がわかったほうが良い場合は上。
  //PVector newTarget2 = q.mult(target);

  // 3. 座標がわかったのでその位置を原点にする（今回の例では利用せず）
  pushMatrix();
  //translate(newTarget.x, newTarget.y, newTarget.z);
  popMatrix();
  
  // 4. 座標軸は以下の通り。赤＝X軸、緑=Y軸、青=Z軸
 
  float vdX,vdY,vdZ;
  float sc=200.0;//線の長さ
  
  //Gyroセンサ姿勢変更後のZ軸成分を内積を用いて計算する
  //Z軸成分とは重力場方向成分のこと  
  
  vdX=sc*(axisZ.dot(newAxisX));//Z軸成分を求める
  vdY=sc*(axisZ.dot(newAxisY));//Z軸成分を求める
  vdZ=sc*(axisZ.dot(newAxisZ));//Z軸成分を求める   
  //println("dot= ",vdX,vdY,vdZ,x_rot); //内積 
  
  stroke(255,0,0);strokeWeight(5);
  line(0,0,0,vdX*newAxisX.x, vdX*newAxisX.y, vdX*newAxisX.z);
  strokeWeight(10);
  point(vdX*newAxisX.x, vdX*newAxisX.y, vdX*newAxisX.z);
  fill(0);textSize(10);text("Accel_X",vdX*newAxisX.x, vdX*newAxisX.y, vdX*newAxisX.z);
  
  stroke(0,255,55);strokeWeight(5);
  line(0,0,0,vdY*newAxisY.x, vdY*newAxisY.y, vdY*newAxisY.z);
  strokeWeight(10);
  point(vdY*newAxisY.x, vdY*newAxisY.y, vdY*newAxisY.z);
  textSize(10);text("Accel_Y",vdY*newAxisY.x, vdY*newAxisY.y, vdY*newAxisY.z);
  
  stroke(0,0,255);strokeWeight(5);
  line(0,0,0,vdZ*newAxisZ.x, vdZ*newAxisZ.y, vdZ*newAxisZ.z);
  strokeWeight(10);
  point(vdZ*newAxisZ.x, vdZ*newAxisZ.y, vdZ*newAxisZ.z);
  textSize(10);text("Accel_Z",vdZ*newAxisZ.x, vdZ*newAxisZ.y, vdZ*newAxisZ.z);
  strokeWeight(1);
  
  //println((vdX*vdX+vdY*vdY+vdZ*vdZ)/sc/sc);/calculate Norm
  
  da+=5.0;

//動画を作る--->gifファイルなどにして使う
/* 
  saveFrame("frames/####.png");   
  if (frameCount >= 120) { // 120コマアニメーションした時
    exit();
  }
*/
}

//*********************************************************************
//矢印を描く
//そらたまご~SORATAMAGO~より
//https://okasho-engineer.com/processing-3d-arrow/
//*********************************************************************


void cone(int L, float radius)
{
  float x, y;
  noStroke();
  beginShape(TRIANGLE_FAN);  // 底面の円の作成
  vertex(0, 0, -L);
  for(float i=0; i<2*PI; )
  {
    x = radius*cos(i);
    y = radius*sin(i);
    vertex(x, y, -L);
    i = i+0.01;
  }
  endShape(CLOSE);
  beginShape(TRIANGLE_FAN);  // 側面の作成
  vertex(0, 0, 0);
  for(float i=0; i<2*PI; )
  {
    x = radius*cos(i);
    y = radius*sin(i);
    vertex(x, y, -L);
    i = i+0.01;
  }
  endShape(CLOSE);
}

void arrow(float x1, float y1, float z1, float x2, float y2, float z2)
{
  int arrowLength = 10;
  float arrowAngle = 0.5;
  float phi = -atan2(y2-y1, x2-x1);
  float theta = PI/2-atan2(z2-z1, x2-x1);
  strokeWeight(4); 
  line(x1, y1, z1, x2, y2, z2);
  strokeWeight(1); 
  pushMatrix();
  translate(x2, y2, z2);
  rotateY(theta);
  rotateX(phi);
  //cone(arrowLength, arrowLength*sin(arrowAngle), Color1, Color2, Color3);
  cone(arrowLength, arrowLength*sin(arrowAngle));
  popMatrix();
}
