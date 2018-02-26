using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;

using System.IO;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using Bespoke.Common;
using Bespoke.Common.Osc;
using System.ComponentModel;



namespace SendHeadPose2
{
   /// <summary>
   /// MainWindow.xaml の相互作用ロジック
   /// </summary>
   public partial class MainWindow : Window
   {

       // kinet sensor object
       private KinectSensor kinectsensor = null;

       // color frame reader object
       private ColorFrameReader colorFrameReader = null;
       private FrameDescription colorFrameDescription = null;

       // body frame reader object
       private BodyFrameReader bodyFrameReader = null;

       // object for storing body detection
       private Body[] bodies = null;

       // face frame source/reader object
       private FaceFrameSource faceFrameSource = null;
       private FaceFrameReader faceFrameReader = null;

       // items for analysing face information
       private const FaceFrameFeatures DefaultFaceFrameFeatures =
                       FaceFrameFeatures.BoundingBoxInColorSpace |
                       FaceFrameFeatures.PointsInColorSpace |
                       FaceFrameFeatures.RotationOrientation |
                       FaceFrameFeatures.FaceEngagement |
                       FaceFrameFeatures.Glasses |
                       FaceFrameFeatures.Happy |
                       FaceFrameFeatures.LeftEyeClosed |
                       FaceFrameFeatures.RightEyeClosed |
                       FaceFrameFeatures.LookingAway |
                       FaceFrameFeatures.MouthMoved |
                       FaceFrameFeatures.MouthOpen;

       public struct Vector3DF
       {
           public float x, y, z;
       }

       // 顔の回転角・位置を格納する
       private Vector3DF rotationXYZ = new Vector3DF();
       private Vector3DF positionXYZ = new Vector3DF();

       // 顔のトラッキングに成功したかどうかを表すフラグ
       //private bool getFrag = false;

       // 送信元・送信先のネットワークエンドポイント
       private IPEndPoint from = null;
       private IPEndPoint toExp = null;
       //private IPEndPoint toListener = null;

       // 送信元・送信先ポート番号
       private int portFrom = 7777;
       private int portTo = 1234;


       /// <summary>
       /// Face rotation display angle increment in degrees
       /// </summary>
       private const double FaceRotationIncrementInDegrees = 0.1;


       public MainWindow()
       {
           InitializeComponent();

           // one sensor is currently supported
           this.kinectsensor = KinectSensor.GetDefault();


           // get the color frame details
           this.colorFrameReader = this.kinectsensor.ColorFrameSource.OpenReader();
           this.colorFrameDescription = this.kinectsensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
           this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

           // get the body frame details
           this.bodyFrameReader = this.kinectsensor.BodyFrameSource.OpenReader();
           this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;
           this.bodies = new Body[this.kinectsensor.BodyFrameSource.BodyCount];

           // get the face frame details
           this.faceFrameSource = new FaceFrameSource(this.kinectsensor, 0, DefaultFaceFrameFeatures);
           this.faceFrameReader = this.faceFrameSource.OpenReader();
           this.faceFrameReader.FrameArrived += this.Reader_FaceFrameArrived;

           this.kinectsensor.Open();

       }




       /// <summary>
       /// Converts rotation quaternion to Euler angles
       /// And then maps them to a specified range of values to control the refresh rate
       /// </summary>
       /// <param name="rotQuaternion">face rotation quaternion</param>
       /// <param name="pitch">rotation about the X-axis</param>
       /// <param name="yaw">rotation about the Y-axis</param>
       /// <param name="roll">rotation about the Z-axis</param>
       private static void ExtractFaceRotationInDegrees(double qx, double qy, double qz, double qw, out double pitch, out double yaw, out double roll)
       {
           double x = qx;
           double y = qy;
           double z = qz;
           double w = qw;

           double pitchD, yawD, rollD;

           // convert face rotation quaternion to Euler angles in degrees

           pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
           yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
           rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

           // clamp the values to a multiple of the specified increment to control the refresh rate
           double increment = FaceRotationIncrementInDegrees;
           pitch = (double)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
           yaw = (double)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
           roll = (double)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
       }


       /// <summary>
       /// Handles the color frame data arriving from the sensor
       /// </summary>
       /// <param name="sender">object sending the event</param>
       /// <param name="e">event arguments</param>
       private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
       {
           // get color frame
           using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
           {
               if (colorFrame != null)
               {
                   FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                   using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                   {
                       //
                   }
               }
           }

       }

       /// <summary>
       /// Handles the body frame data arriving from the sensor
       /// </summary>
       /// <param name="sender">object sending the event</param>
       /// <param name="e">event arguments</param>
       private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
       {

           using (var bodyFrame = e.FrameReference.AcquireFrame())
           {
               if (bodyFrame != null)
               {
                   // update body data
                   bodyFrame.GetAndRefreshBodyData(this.bodies);
                   foreach (Body body in this.bodies)
                   {
                       if (body.IsTracked)
                       {
                           this.faceFrameSource.TrackingId = body.TrackingId;
                           Joint head = body.Joints[JointType.Head];

                           positionXYZ.x = head.Position.X;
                           positionXYZ.y = head.Position.Y;
                           positionXYZ.z = head.Position.Z;

                           break;
                       }
                   }
               }
           }

       }

       /// <summary>
       /// Handles the face frame data arriving from the sensor
       /// </summary>
       /// <param name="sender">object sending the event</param>
       /// <param name="e">event arguments</param>
       private void Reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
       {
           double yaw, pitch, roll;

           using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
           {
               if (faceFrame != null)
               {
                   if (faceFrame.FaceFrameResult != null)
                   {
                       double qx = (double)faceFrame.FaceFrameResult.FaceRotationQuaternion.X;
                       double qy = (double)faceFrame.FaceFrameResult.FaceRotationQuaternion.Y;
                       double qz = (double)faceFrame.FaceFrameResult.FaceRotationQuaternion.Z;
                       double qw = (double)faceFrame.FaceFrameResult.FaceRotationQuaternion.W;

                       ExtractFaceRotationInDegrees(qx, qy, qz, qw, out pitch, out yaw, out roll);

                       rotationXYZ.x = (float)pitch;
                       rotationXYZ.y = (float)yaw;
                       rotationXYZ.z = (float)roll;

                       sendOSCMessage(rotationXYZ, positionXYZ);

                       // トラッキングの状態を表示
                       StatusTextBlock.Text = "Status: Tracked";
                   }
                   else
                   {
                       StatusTextBlock.Text = "Status: Not Tracked";
                   }
               }
           }
       }



       /// <summary>
       /// 初期化処理
       /// </summary>
       /// <param name="sender"></param>
       /// <param name="e"></param>
       private void Window_Loaded(object sender, RoutedEventArgs e)
       {

           // ソケットの生成
           from = new IPEndPoint(IPAddress.Loopback, portFrom);
           toExp = new IPEndPoint(IPAddress.Loopback, portTo);
           //            toLisner = new IPEndPoint(IPAddress.Parse("192.168.11.15"), portTo);
           //            toLisner = new IPEndPoint(IPAddress.Parse("192.168.11.7"), portTo);
           //            toLisner = new IPEndPoint(IPAddress.Loopback, portTo);

           //            from = new IPEndPoint(IPAddress.Parse("192.168.11.24"), portFrom);
           //            to = new IPEndPoint(IPAddress.Parse("192.168.11.21"), portTo);

           // 送信先のアドレスとポート番号の表示
           SendTextBlock.Text = "Target: " + toExp.Address.ToString() + ':' + toExp.Port.ToString();
           //            SendLisnerTextBlock.Text = "送信先(受聴者): " + toLisner.Address.ToString() + ':' + toLisner.Port.ToString();

           /*
           // KinectChooserにイベントハンドラの登録、KinectChooserの開始
           kinectChooser.KinectChanged += new EventHandler<KinectChangedEventArgs>(kinectChooser_KinectChanged);
           kinectChooser.Start();
           */

           IPPortUpdate.Click += new RoutedEventHandler(this.IPPortUpdate_Click);



       }

       private void IPPortUpdate_Click(object sender, RoutedEventArgs e)
       {
           IPAddress IPnew = IPAddress.Parse(IPupdate.Text);
           int Portnew = Int32.Parse(PortUpdate.Text);

           toExp = new IPEndPoint(IPnew, Portnew);
           SendTextBlock.Text = "Target: " + toExp.Address.ToString() + ':' + toExp.Port.ToString();

       }


       private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
       {
           if (bodyFrameReader != null)
           {
               bodyFrameReader.Dispose();
               bodyFrameReader = null;
           }

           if (kinectsensor != null)
           {
               kinectsensor.Close();
               kinectsensor = null;
           }

       }

       /// <summary>
       /// 終了処理
       /// </summary>
       /// <param name="sender"></param>
       /// <param name="e"></param>
       private void Window_Closed(object sender, EventArgs e)
       {
           //kinectChooser.Stop();
       }



       /*
       /// <summary>
       /// 頭部の回転角・位置の取得を行い、データを送信する
       /// </summary>
       /// <param name="sender"></param>
       /// <param name="e"></param>
       void kinect_AllFramesReady(object sender, AllFramesReadyEventArgs e)
       {
           KinectSensor kinect = sender as KinectSensor;

           // 全てのフレームを準備
           using (ColorImageFrame colorFrm = e.OpenColorImageFrame())
           using (DepthImageFrame depthFrm = e.OpenDepthImageFrame())
           using (SkeletonFrame skeletonFrm = e.OpenSkeletonFrame())
           {
               // 全てのフレームが揃っていない場合何もしない
               if (colorFrm == null || depthFrm == null || skeletonFrm == null)
                   return;

               // 各フレームの情報をバッファにコピー
               colorFrm.CopyPixelDataTo(pixelBuffer);
               depthFrm.CopyPixelDataTo(depthBuffer);
               skeletonFrm.CopySkeletonDataTo(skeletonBuffer);

               // 顔の回転角・位置をを求める
               getFrag = false;    // フラグの初期化
               foreach (Body body in bodies)
               {
                   // トラッキングできていない骨格は処理しない
                   if (ske.TrackingState != SkeletonTrackingState.Tracked)
                       continue;

                   SkeletonPoint skePosition = ske.Position;
                   foreach (Joint joint in ske.Joints)
                   {
                       JointType jointType = joint.JointType;
                       SkeletonPoint position = joint.Position;

                       if (joint.TrackingState != JointTrackingState.NotTracked)
                       {
                           if (jointType == JointType.HandRight)
                           {
                               righthand.X = position.X;
                               righthand.Y = position.Y;
                               righthand.Z = position.Z;
                           }
                           else if (jointType == JointType.HandLeft)
                           {
                               lefthand.X = position.X;
                               lefthand.Y = position.Y;
                               lefthand.Z = position.Z;
                           }
                           else if (jointType == JointType.ElbowRight)
                           {
                               rightelbow.X = position.X;
                               rightelbow.Y = position.Y;
                               rightelbow.Z = position.Z;
                           }
                           else if (jointType == JointType.ElbowLeft)
                           {
                               leftelbow.X = position.X;
                               leftelbow.Y = position.Y;
                               leftelbow.Z = position.Z;
                           }

                       }


                   }



                   // 今回のフレームにFaceTrackingを適用
                   FaceTrackFrame faceFrm = faceTracker.Track(rgbFormat, pixelBuffer, depthFormat, depthBuffer, ske);

                   // 顔のトラッキングが成功していれば、回転角・位置を取得し、送信
                   if (faceFrm.TrackSuccessful)
                   {
                       faceFrm.getResult(out rotationXYZ, out positionXYZ);
                       sendOSCMessage(rotationXYZ, positionXYZ, righthand, rightelbow, lefthand, leftelbow);
                       getFrag = true;
                       break;
                   }
               }

               // トラッキングの状態を表示
               if (getFrag)
                   StatusTextBlock.Text = "Status: Tracked";
               else
                   StatusTextBlock.Text = "Status: Not Tracked";
               // 受聴者側へトラッキング状態を送信
               //                sendTrackingState(getFrag);
           }
       }
       */


       /// <summary>
       /// 取得した回転角・位置情報をOSCを用いて送信
       /// </summary>
       /// <param name="rotBuff">顔の位置情報</param>
       /// <param name="posBuff">顔の回転角</param>
       void sendOSCMessage(Vector3DF rotBuff, Vector3DF posBuff) //, Vector3DF rhBuff, Vector3DF reBuff, Vector3DF lhBuff, Vector3DF leBuff)
       {
           // OSCメッセージの生成
           OscMessage rotOSC = new OscMessage(from, "/rotation", null);
           OscMessage posOSC = new OscMessage(from, "/position", null);
           //OscMessage rhOSC = new OscMessage(from, "/hand_r", null);
           //OscMessage lhOSC = new OscMessage(from, "/hand_l", null);
           //OscMessage reOSC = new OscMessage(from, "/elbow_r", null);
           //OscMessage leOSC = new OscMessage(from, "/elbow_l", null);


           // 情報の追加
           rotOSC.Append<float>(rotBuff.x);
           rotOSC.Append<float>(rotBuff.y);
           rotOSC.Append<float>(rotBuff.z);
           posOSC.Append<float>(posBuff.x);
           posOSC.Append<float>(posBuff.y);
           posOSC.Append<float>(posBuff.z);
           /*
           rhOSC.Append<float>(rhBuff.x);
           rhOSC.Append<float>(rhBuff.y);
           rhOSC.Append<float>(rhBuff.z);
           lhOSC.Append<float>(lhBuff.x);
           lhOSC.Append<float>(lhBuff.y);
           lhOSC.Append<float>(lhBuff.z);
           reOSC.Append<float>(reBuff.x);
           reOSC.Append<float>(reBuff.y);
           reOSC.Append<float>(reBuff.z);
           leOSC.Append<float>(leBuff.x);
           leOSC.Append<float>(leBuff.y);
           leOSC.Append<float>(leBuff.z);
            */


           // 送信
           rotOSC.Send(toExp);
           posOSC.Send(toExp);
           /*
           rhOSC.Send(toExp);
           reOSC.Send(toExp);
           lhOSC.Send(toExp);
           leOSC.Send(toExp);
           */


       }

   }

}