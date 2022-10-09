# MPU-6050
単なる動作確認用プログラムです。多くの思い違いや明らかなミスなどあるかと思います。 参考程度でご利用ください。皆様のわずかなりのご参考になれば幸いです。

https://youtu.be/esXm_ZsbzWM

（１）program：MPU6050による姿勢の検出を行う。C言語。以下に下条が既存プログラムに変更を加えた点を示す。

Madgwick Filterへのサンプリング周波数の受渡（Madgwick.cの一部改変含む）を行う。 MPU6050でのサンプリング周期の計測を入れた。 MPU6050青ボタンによる割込み処理でGyroのオフセットの補正を行う。 描画ソフトprocessingへのデータ出力（Teraterm）。

（２）Rotation_shuttleXYZ： processingでの描画プログラム。

　Teraterm経由でMPU6050からのデータを受け取り、IMUの動きに応じてshuttleを回転する。

（３）Accel_PostureQuaternion_NewV2：　processingでの描画プログラム、MPU6050の制御とは無関係です。

　ジャイロセンサの姿勢とその時の加速度センサの出力を表示する単独のプログラムです。

（４）Mdgwic4PushButton_final3.pdf

MPU6050+Madgwichによる姿勢の検出を行うプログラムのメモです。

なお、

プログラム作成後、数週間経過するとプログラムの詳細が分からなくなりました。年齢のためか（71歳）。このため、プログラム等に対する問い合わせは回答出来ません。あしからず。

以上です

