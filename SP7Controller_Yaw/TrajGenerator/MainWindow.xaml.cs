using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Fune;
using System.Threading;
using System.Globalization;
using System.Diagnostics;
using Microsoft.Win32;
using SharpDX.DirectInput;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using System.Net.Sockets;
using System.Net;
using System.IO;

namespace SP7
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        struct ControllerDefinition
        {
            public int DeadZone;
            public int Saturation;
            public JoystickOffset[] Axis;
            public UInt32[] AxisButtonMask;
            public UInt32[] AxisButtonState;
            public bool[] AxisInvert;
            public UInt32 ZeroButtonMask;
            public UInt32 LockRButtonMask;
            public UInt32 LockTButtonMask;
        }

        readonly ControllerDefinition Mouse3D = new ControllerDefinition()
        {
            DeadZone = 15,
            Saturation = 350,
            Axis = new JoystickOffset[] { JoystickOffset.X, JoystickOffset.Y, JoystickOffset.Z, JoystickOffset.RotationY, JoystickOffset.RotationX, JoystickOffset.RotationZ },
            AxisButtonMask = new UInt32[] { 0, 0, 0, 0, 0, 0 },
            AxisButtonState = new UInt32[] { 0, 0, 0, 0, 0, 0 },
            AxisInvert = new bool[] { true, true, true, true, true, false },
            ZeroButtonMask = 0x00000001,
            LockRButtonMask = 0x00000040,
            LockTButtonMask = 0x00000080,
        };

        readonly ControllerDefinition CommonJoystick = new ControllerDefinition()
        {
            DeadZone = 5,
            Saturation = 128,
            Axis = new JoystickOffset[] { JoystickOffset.X, JoystickOffset.Y, JoystickOffset.X, JoystickOffset.X, JoystickOffset.Y, JoystickOffset.X },
            AxisButtonMask = new UInt32[] { 0x3, 0x3, 0x3, 0x3, 0x3, 0x1 },
            AxisButtonState = new UInt32[] { 0x2, 0x2, 0x3, 0x0, 0x0, 0x0 },
            AxisInvert = new bool[] { false, false, false, false, false, false },
            ZeroButtonMask = 0x00000001,
        };

        const int plugin_port = 0x5850;

        ManualResetEvent _statusUpdate = new ManualResetEvent(false);
        SP7Controller _sp7 = null;
        SP7Controller.MotionData _mData = null;
        Timer _timer;
        TextBlock[] VelBoxes;
        TextBlock[] PosBoxes;
        TextBlock[] DriveStateBoxes;
        Joystick joystick;
        ControllerDefinition curControllerDefinition;

        StreamWriter sw;
        Object StreamSyncObj = new Object();

        QuaternionRotation3D q3d = new QuaternionRotation3D();
/*        AxisAngleRotation3D R3DYaw = new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0);
        AxisAngleRotation3D R3DPitch = new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0);
        AxisAngleRotation3D R3DRoll = new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0); */
        TranslateTransform3D T3D = new TranslateTransform3D();

        bool feeding = false;
        Thread feedThread = null;
        List<Tuple<double, SP7Controller.SP7Pose, SP7Controller.SP7Vel, double>> _samples = new List<Tuple<double, SP7Controller.SP7Pose, SP7Controller.SP7Vel, double>>();

        public MainWindow()
        {
            InitializeComponent();
            UpdateControls();
            _timer = new Timer(UpdateData, null, 100, 100);
            VelBoxes = new TextBlock[] { VelBox1, VelBox2, VelBox3, VelBox4, VelBox5, VelBox6, VelBox7 };
            PosBoxes = new TextBlock[] { PosBox1, PosBox2, PosBox3, PosBox4, PosBox5, PosBox6, PosBox7 };
            DriveStateBoxes = new TextBlock[] { DStateBox1, DStateBox2, DStateBox3, DStateBox4, DStateBox5, DStateBox6, DStateBox7 };


            var directInput = new DirectInput();
            joystick = null;
            // try looking for 3d 
            foreach (var deviceInstance in directInput.GetDevices(DeviceType.FirstPerson, DeviceEnumerationFlags.AllDevices))
            {
                if (deviceInstance.InstanceName.Equals("SpaceMouse Pro"))
                {
                    joystick = new Joystick(directInput, deviceInstance.InstanceGuid);
                    curControllerDefinition = Mouse3D;
                    break;
                }
            }

            if (joystick == null)
            {
                // try with generic joystick
                foreach (var deviceInstance in directInput.GetDevices(DeviceType.Joystick, DeviceEnumerationFlags.AllDevices))
                {
                    joystick = new Joystick(directInput, deviceInstance.InstanceGuid);
                    curControllerDefinition = CommonJoystick;
                    break;
                }
            }

            if (joystick != null)
                ControllerFeederCB.IsChecked = true;

            // 3D
            var objr = new ObjReader();
            Model3DGroup endEffectorModel = objr.Read(@"models\endeffector.obj");
            Transform3DGroup tg = new Transform3DGroup();
            /*            tg.Children.Add(new RotateTransform3D(R3DYaw));
                        tg.Children.Add(new RotateTransform3D(R3DPitch));
                        tg.Children.Add(new RotateTransform3D(R3DRoll)); */
            tg.Children.Add(new RotateTransform3D(q3d));
            tg.Children.Add(T3D);
            endEffectorModel.Transform = tg;
            SceneModel.Children.Add(endEffectorModel); 
            objr = new ObjReader();
            SceneModel.Children.Add(objr.Read(@"models\platform.obj"));
        }


        public void UpdateControls()
        {
            try
            {
                Dispatcher.Invoke(() =>
                {
                    bool connected = (_sp7 != null) && ((_sp7.IsConnected));
                    ConnBtn.IsEnabled = !connected;
                    DiscBtn.IsEnabled = connected;
                    IpBox.IsEnabled = !connected && !SimulateCB.IsChecked.Value;

                    OpenBtn.IsEnabled = connected & !feeding;
                    FeedBtn.IsEnabled = connected && !feeding;
                    StopFeedBtn.IsEnabled = connected && feeding;
                    HaltBtn.IsEnabled = connected;
                    ResetBtn.IsEnabled = connected;
                    FreqBox.IsEnabled = connected && !feeding;
                    AmplitudeBox.IsEnabled = connected && !feeding;
                    Continuous.IsEnabled = connected && !feeding;
                    Synchronized.IsEnabled = connected && !feeding;
                    AngAmplitudeBox.IsEnabled = connected && !feeding;

                    XEn.IsEnabled = YEn.IsEnabled = ZEn.IsEnabled = connected && !feeding;
                    RollEn.IsEnabled = PitchEn.IsEnabled = connected && !feeding;
                });
            }
            catch (Exception) { }
        }

        private void UpdateTargetPose(SP7Controller.SP7Pose x, SP7Controller.SP7Vel xdot)
        {
            try
            {
                Dispatcher.Invoke(() =>
                {
                    TargetX.Text = $"{x.x:0.000}";
                    TargetY.Text = $"{x.y:0.000}";
                    TargetZ.Text = $"{x.z:0.000}";
                    TargetRoll.Text = $"{(int)(x.roll * 180.0 / Math.PI)}";
                    TargetPitch.Text = $"{(int)(x.pitch * 180.0 / Math.PI)}";
                    TargetYaw.Text = $"{(int)(x.yaw * 180.0 / Math.PI)}";
                });
            }
            catch (Exception) { }
        }


        public void UpdateData(object ctx)
        {
            try
            {
                Dispatcher.Invoke(() =>
                {
                    if (_mData == null || _sp7 == null || !_sp7.IsConnected)
                    {
                        for (int joint = 0; joint < 7; joint++)
                        {
                            VelBoxes[joint].Text = "";
                            PosBoxes[joint].Text = "";
                            DriveStateBoxes[joint].Text = "";
                        }
                        SystemStateBox.Text = "";
                        MotionStateBox.Text = "";
                        return;
                    }
                    SP7Controller.MotionData md = _mData;

                    SystemStateBox.Text = md.SState.ToString();
                    MotionStateBox.Text = md.MState.ToString();
                    for (int joint = 0; joint < 7; joint++)
                    {
                        VelBoxes[joint].Text = String.Format("{0:0.0}", md.JointVelocity[joint] * 180.0 / Math.PI);
                        PosBoxes[joint].Text = String.Format("{0:0.0}", md.JointPosition[joint] * 180.0 / Math.PI);
                        DriveStateBoxes[joint].Text = md.DState[joint].ToString();
                    }
                    if ((md.IKState != 0) || (md.FKState != 0))
                    {
                        StringBuilder sb = new StringBuilder();
                        if (md.IKState != 0)
                            sb.Append($"IK error:{(int)md.IKState}");
                        if (md.FKState != 0)
                            sb.Append($"FK error:{(int)md.FKState}");
                        KinematicsStateBox.Text = sb.ToString();
                    } else
                    {
                        KinematicsStateBox.Text = "OK";
                    }
                });
            }
            catch (Exception) { }
        }


        private void Update3D(SP7Controller.SP7Pose x)
        {
            Dispatcher.Invoke(() =>
            {
                // Abbreviations for the various angular functions
                double cy = Math.Cos(x.yaw * 0.5);
                double sy = Math.Sin(x.yaw * 0.5);
                double cr = Math.Cos(x.roll * 0.5);
                double sr = Math.Sin(x.roll * 0.5);
                double cp = Math.Cos(x.pitch * 0.5);
                double sp = Math.Sin(x.pitch * 0.5);

                Quaternion q = new Quaternion(
                    cy * sr * cp - sy * cr * sp,
                    cy * cr * sp + sy * sr * cp,
                    sy * cr * cp - cy * sr * sp,
                    cy * cr * cp + sy * sr * sp);

                /*                R3DYaw.Angle = x.yaw * 180.0 / Math.PI;
                                R3DPitch.Angle = x.pitch * 180.0 / Math.PI;
                                R3DRoll.Angle = x.roll * 180.0 / Math.PI; */
                q3d.Quaternion = q;
                T3D.OffsetX = (float)x.x;
                T3D.OffsetY = (float)x.y;
                T3D.OffsetZ = (float)x.z + 0.2;
            });
        }

        private void MotionDataReceived(object sender, SP7Controller.MotionData md)
        {
            _mData = md;
            lock (StreamSyncObj)
            {
                if (sw != null)
                {
                    sw.Write($"{md.Timestamp},"); // time here
                    for (int i = 0; i < 7; i++)
                    {
                        sw.Write($"{_mData.JointPosition[i].ToString(CultureInfo.InvariantCulture)}, {_mData.JointVelocity[i].ToString(CultureInfo.InvariantCulture)},");
                        sw.Write($"{_mData.JointRefPosition[i].ToString(CultureInfo.InvariantCulture)}, {_mData.JointRefVelocity[i].ToString(CultureInfo.InvariantCulture)},");
                    }
                    for (int i = 0; i < 6; i++)
                    {
                        sw.Write($"{_mData.PlatTargetPosition[i].ToString(CultureInfo.InvariantCulture)}, {_mData.PlatTargetVelocity[i].ToString(CultureInfo.InvariantCulture)}");
                        if (i < 5)
                            sw.Write(",");
                    }
                
                    DateTimeOffset offset = new DateTimeOffset(DateTime.Now);
                    var unixTimestamp = offset.ToUnixTimeMilliseconds();
                    sw.Write($"{unixTimestamp.ToString(CultureInfo.InvariantCulture)}");

                    sw.WriteLine();
                }
            }
            _statusUpdate.Set();
        }


        private void ConnectionStateChanged(object sender, bool connected)
        {
            if (false)
                MessageBox.Show("Device disconnected", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            lock (StreamSyncObj)
            {
                if (connected)
                {
                    DateTime now = DateTime.Now;
                    sw = new StreamWriter($"SP7MotionData-{now:yyyyMMddHHmmss}.log");
                     sw.Write("#Timestamp,q1,qdot1,qref1,qdotref1,q2,qdot2,qref2,qdotref2,q3,qdot3,qref3,qdotref3,q4,qdot4,qref4,qdotref4,q5,qdot5,qref5,qdotref5,q6,qdot6,qref6,qdotref6,q7,qdot7,qref7,qdotref7,");
                    sw.WriteLine("p1,pdot1,p2,pdot2,p3,pdot3,p4,pdot4,p5,pdot5,p6,pdot6, #sEpochTimeStampMS");

                }
                else
                {
                    if (sw != null)
                        sw.Close();
                    sw = null;
                }
            }
            UpdateControls();
        }

        private void ConnBtn_Click(object sender, RoutedEventArgs e)
        {
            _sp7 = new SP7Controller(IpBox.Text, SimulateCB.IsChecked.Value);
            _sp7.ConnectionStateChanged += ConnectionStateChanged;
            if (!_sp7.Connect())
            {
                _sp7.ConnectionStateChanged -= ConnectionStateChanged;
                _sp7 = null;
                MessageBox.Show("Unable to connect", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
            else
            {
                _sp7.MotionDataReceived += MotionDataReceived;
            }
            UpdateControls();
        }

        private void DiscBtn_Click(object sender, RoutedEventArgs e)
        {
            if (_sp7 != null)
            {
                StopFeeder();
                _sp7.MotionDataReceived -= MotionDataReceived;
                _sp7.ConnectionStateChanged -= ConnectionStateChanged;
                _sp7.Disconnect();
            }
            _sp7 = null;
            UpdateControls();
        }


        private void FeedBtn_Click(object sender, RoutedEventArgs e)
        {
            StartFeeder();
        }

        private void StopFeedBtn_Click(object sender, RoutedEventArgs e)
        {
            StopFeeder();
        }

        private void OpenBtn_Click(object sender, RoutedEventArgs e)
        {
            FileFeederCB.IsChecked = true;
            OpenFileDialog fd = new OpenFileDialog();
            if (fd.ShowDialog() == true)
            {
                FilenameBlock.Text = "";
                if (ReadSamples(fd.FileName))
                {
                    FilenameBlock.Text = fd.FileName;
                }
                else
                {
                    MessageBox.Show("Invalid data format", "Error",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
        }

        private void HaltBtn_Click(object sender, RoutedEventArgs e)
        {
            _sp7.Halt();
        }

        private void ResetBtn_Click(object sender, RoutedEventArgs e)
        {
            _sp7.Reset();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if ((_sp7 != null) && (_sp7.IsConnected))
                _sp7.Disconnect();
        }

        private void SimulateCB_Checked(object sender, RoutedEventArgs e)
        {
            UpdateControls();
        }

        void StartFeeder()
        {
            if (feeding)
                return;
            double a, b, f;
            if (!Double.TryParse(AmplitudeBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out a))
            {
                MessageBox.Show("Invalid motion amplitude", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!Double.TryParse(AngAmplitudeBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out b))
            {
                MessageBox.Show("Invalid motion angular amplitude", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!Double.TryParse(FreqBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out f))
            {
                MessageBox.Show("Invalid motion frequency", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            b = b * Math.PI / 180.0;
            
            double yawSpeed, duration;
            if (!Double.TryParse(NumberOfRevolutionBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out yawSpeed))
            {
                MessageBox.Show("Invalid Number of revolution", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            if (!Double.TryParse(DurationBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out duration))
            {
                MessageBox.Show("Invalid Duration", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            if (XYFeederCB.IsChecked.HasValue && XYFeederCB.IsChecked.Value)
            {
                bool[] enabled = { XEn.IsChecked.Value, YEn.IsChecked.Value, ZEn.IsChecked.Value,
                    RollEn.IsChecked.Value,PitchEn.IsChecked.Value };

                feedThread = new Thread(() => { xyFeeder(a, b, f, enabled); });
            }
            else if (FileFeederCB.IsChecked.HasValue && FileFeederCB.IsChecked.Value)
            {
                bool cont = Continuous.IsChecked.HasValue && Continuous.IsChecked.Value;
                bool sync = Synchronized.IsChecked.HasValue && Synchronized.IsChecked.Value;
                feedThread = new Thread(() => { trajFeeder(cont, sync); });
            }
            else if (ControllerFeederCB.IsChecked.HasValue && ControllerFeederCB.IsChecked.Value)
            {
                feedThread = new Thread(() => { controllerFeeder(curControllerDefinition); });
            }
            else if (SimFeederCB.IsChecked.HasValue && SimFeederCB.IsChecked.Value)
            {
                feedThread = new Thread(() => { simFeeder(plugin_port); });
            }
            else if(YawRotationFeederCB.IsChecked.HasValue && YawRotationFeederCB.IsChecked.Value)
            {
                bool negDirection = NegativeDirCK.IsChecked.Value;
                feedThread = new Thread(() => { yawRotationFeeder(yawSpeed, duration, negDirection); });
            }
            feedThread.Start();
            UpdateControls();
        }

        private void yawRotationFeeder(double speed, double duration, bool negDirection)
        {
            feeding = true;
            UpdateControls();

            SP7Controller.SP7JointPos j;
            SP7Controller.SP7JointVel jdot;

            long ms;


            // Zeroing in joint control
            j.ax1 = j.ax2 = j.ax3 = j.ax4 = j.ax5= j.ax6 = j.ax7 = 0 * Math.PI / 180.0;
            jdot.vax1 = jdot.vax2 = jdot.vax3 = jdot.vax4 = jdot.vax5= jdot.vax6 = jdot.vax7 = 0;
             _sp7.SetJointTargets(j, jdot);
            Thread.Sleep(500);

            speed *= Math.PI / 180.0;//Deg2Rad
            if (negDirection)
            {
                speed = -speed;
            }

            Stopwatch stopwatch = Stopwatch.StartNew();
            ms = stopwatch.ElapsedMilliseconds;
            long lastms = ms;
            while (feeding && _sp7.IsConnected)
            {
                ms = stopwatch.ElapsedMilliseconds;
                long dtms = ms - lastms;
                lastms = ms;
                double t = ((double)ms) / 1000.0;

                Dispatcher.Invoke(() =>
                {
                    PlaybackTimeLbl.Text = $"{t:0000.000}";
                });

                if (t <= duration)
                {
                    jdot.vax7 = speed;
                    j.ax7 = jdot.vax7 * t;
                    _sp7.SetJointTargets(j, jdot);
                }
                else
                {
                    stopwatch.Stop();
                    feeding = false;
                    Thread.Sleep(500);
                    jdot.vax1 = jdot.vax2 = jdot.vax3 = jdot.vax4 = jdot.vax5 = jdot.vax6 = jdot.vax7 = 0;
                    _sp7.SetJointTargets(j, jdot);
                    Thread.Sleep(500);
                    UpdateControls();
                    //_sp7.Reset();//uncomment this if you want the platform to return to zero
                }
                Thread.Sleep(1000 / 60);
            }
        }

        void StopFeeder()
        {
            if (!feeding)
                return;
            feeding = false;
        }

        void xyFeeder(double a, double b, double f_freq, bool[] enabled)
        {
            feeding = true;
            UpdateControls();

            SP7Controller.SP7Pose x;
            SP7Controller.SP7Vel xdot;
            bool flipyaw = false;
            long ms;

            x.x = x.y = x.roll = x.pitch = x.yaw = 0;
            x.z = SP7Controller.Z_ZERO;
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;

            Stopwatch stopwatch = Stopwatch.StartNew();
            double setuptimems = 2000.0;
            while (feeding && _sp7.IsConnected)
            {
                ms = stopwatch.ElapsedMilliseconds;
                if (ms > setuptimems)
                    break;
                if (enabled[0])
                {
                    x.x = 0;
                    xdot.vx = 0;
                }
                if (enabled[1])
                {
                    x.y = a * ms / setuptimems;
                    xdot.vy = a * 1000.0 / setuptimems;
                }
                if (enabled[2])
                {
                    x.z = SP7Controller.Z_ZERO;
                    xdot.vz = 0;
                }
                if (enabled[3])
                {
                    x.roll = 0;
                    xdot.vroll = 0;
                }
                if (enabled[4])
                {
                    x.pitch = b * ms / setuptimems;
                    xdot.vpitch = a * 1000.0 / setuptimems;
                }
                _sp7.SetEETargets(x, xdot);
                UpdateTargetPose(x, xdot);
                Thread.Sleep(1000 / 60);
            }
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);

            Thread.Sleep(500);

            stopwatch.Reset();
            stopwatch.Start();
            double yawspeed = 10 * Math.PI / 180.0;
            ms = stopwatch.ElapsedMilliseconds;
            long lastms = ms;
            while (feeding && _sp7.IsConnected)
            {
                ms = stopwatch.ElapsedMilliseconds;
                long dtms = ms - lastms;
                lastms = ms;

                double t = ((double)ms) / 1000.0;
                double wt = 2.0 * Math.PI * f_freq * t;
                double dwt_dt = 2.0 * Math.PI * f_freq;
                double swt = Math.Sin(wt);
                double cwt = Math.Cos(wt);

                if (enabled[0])
                {
                    x.x = a * swt;
                    xdot.vx = a * dwt_dt * cwt;
                }
                if (enabled[1])
                {
                    x.y = a * cwt;
                xdot.vy = a * dwt_dt * -swt;
                }
                if (enabled[2])
                {
                    x.z = SP7Controller.Z_ZERO + a * swt;
                xdot.vz = a * dwt_dt * cwt;
                }
                if (enabled[3])
                {
                    x.roll = b * swt;
                xdot.vroll = b * dwt_dt * cwt;
                }
                if (enabled[4])
                {
                    x.pitch = b * swt;
                    xdot.vpitch = b * dwt_dt * cwt;
                }
                _sp7.SetEETargets(x, xdot);
                UpdateTargetPose(x, xdot);

                /*
                if (flipyaw)
                {
                    x.yaw -= yawspeed * dtms / 1000.0;
                    xdot.vyaw = -yawspeed;
                    if (x.yaw < (-90.0 * Math.PI / 180.0))
                        flipyaw = false;
                }
                else
                {
                    x.yaw += yawspeed * dtms / 1000.0;
                    xdot.vyaw = yawspeed;
                    if (x.yaw > (90 * Math.PI / 180.0))
                        flipyaw = true;
                }
                */
                try
                {
                    Update3D(x);
                }
                catch (Exception ex) { }

                Thread.Sleep(1000 / 60);
            }

            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);
            stopwatch.Stop();
            Thread.Sleep(100);
            _sp7.Reset();
            feeding = false;
            UpdateControls();
        }

        void yawFeeder(double a, double b, double f_freq)
        {
            feeding = true;
            UpdateControls();

            SP7Controller.SP7Pose x;
            SP7Controller.SP7Vel xdot;
            bool flipyaw = false;
            long ms;

            x.x = x.y = x.roll = x.pitch = x.yaw = 0;
            x.z = SP7Controller.Z_ZERO;
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;

            _sp7.SetEETargets(x, xdot);

            Thread.Sleep(500);

            Stopwatch stopwatch = Stopwatch.StartNew();
            double yawspeed = 10 * Math.PI / 180.0;
            ms = stopwatch.ElapsedMilliseconds;
            long lastms = ms;
            while (feeding && _sp7.IsConnected)
            {
                ms = stopwatch.ElapsedMilliseconds;
                long dtms = ms - lastms;
                lastms = ms;

                double t = ((double)ms) / 1000.0;

                xdot.vyaw = 10.0 * Math.PI / 180.0;
                x.yaw = xdot.vyaw * t;

                _sp7.SetEETargets(x, xdot);
                UpdateTargetPose(x, xdot);

                if (flipyaw)
                {
                    x.yaw -= yawspeed * dtms / 1000.0;
                    xdot.vyaw = -yawspeed;
                    if (x.yaw < (-90.0 * Math.PI / 180.0))
                        flipyaw = false;
                }
                else
                {
                    x.yaw += yawspeed * dtms / 1000.0;
                    xdot.vyaw = yawspeed;
                    if (x.yaw > (90 * Math.PI / 180.0))
                        flipyaw = true;
                }

                try
                {
                    Update3D(x);
                }
                catch (Exception ex) { }

                Thread.Sleep(1000 / 60);
            }

            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);
            stopwatch.Stop();
            Thread.Sleep(100);
            _sp7.Reset();
            feeding = false;
            UpdateControls();
        }

        void trajFeeder(bool continuous, bool sync)
        {
            feeding = true;
            UpdateControls();

            SP7Controller.SP7Pose x;
            SP7Controller.SP7Vel xdot;

            x.x = x.y = x.roll = x.pitch = x.yaw = 0;
            x.z = SP7Controller.Z_ZERO;
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;

            if (_samples.Count() == 0)
                return;

            int idx = 0;
            double last_t = 0;

            if (sync)
            {
                double zulu0 = _samples[0].Item4 + 2.0;
                continuous = false;
                while (_samples[idx].Item4 < zulu0)
                {
                    last_t = _samples[idx].Item1;
                    idx++;
                }

                Dispatcher.Invoke(() => {
                    PlaybackTimeLbl.Text = $"Wait For sync";
                });

                UdpClient channel = new UdpClient(plugin_port);
                IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
                double zulu = 0;

                while (feeding && _sp7.IsConnected && (zulu < zulu0))
                {
                    var asyncResult = channel.BeginReceive(null, null);
                    asyncResult.AsyncWaitHandle.WaitOne(100);

                    if (asyncResult.IsCompleted)
                    {
                        byte[] simdata = channel.EndReceive(asyncResult, ref remote);
                        MemoryStream ms = new MemoryStream(simdata);
                        BinaryReader mr = new BinaryReader(ms);
                        float t = mr.ReadSingle();
                        float ax = mr.ReadSingle();
                        float ay = mr.ReadSingle();
                        float az = mr.ReadSingle();
                        float pitch = mr.ReadSingle();
                        float yaw = mr.ReadSingle();
                        float roll = mr.ReadSingle();
                        float vpitch = mr.ReadSingle();
                        float vyaw = mr.ReadSingle();
                        float vroll = mr.ReadSingle();
                        float GS = mr.ReadSingle();
                        float mass = mr.ReadSingle();
                        float ph_x = mr.ReadSingle();
                        float ph_y = mr.ReadSingle();
                        float ph_z = mr.ReadSingle();
                        float ph_roll = mr.ReadSingle();
                        float ph_pitch = mr.ReadSingle();
                        float ph_yaw = mr.ReadSingle();
                        zulu = mr.ReadSingle();
                        Dispatcher.Invoke(() => {
                            PlaybackTimeLbl.Text = $"Wait For sync {(zulu0 - zulu):0.0}";
                        });
                    }
                }
            }

            while (feeding && _sp7.IsConnected)
            {
                if (idx >= _samples.Count)
                {
                    idx = 0;
                    last_t = 0;
                    if (!continuous)
                        break;
                }

                Tuple<double, SP7Controller.SP7Pose, SP7Controller.SP7Vel, double> t = _samples[idx];
                Thread.Sleep((int)((t.Item1 - last_t) * 1000));
                x = t.Item2;
                xdot = t.Item3;
                _sp7.SetEETargets(t.Item2, t.Item3);
                UpdateTargetPose(t.Item2, t.Item3);
                try
                {
                    Update3D(x);
                }
                catch (Exception ex) { }

                last_t = t.Item1;
                idx++;
                if (idx % 10 == 0)
                {
                    Dispatcher.Invoke(() =>
                    {
                        PlaybackTimeLbl.Text = $"{last_t:0000.0}";
                    });
                }

            }
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);
            Thread.Sleep(100);
            _sp7.Reset();
            feeding = false;
            UpdateControls();
        }


        private bool ReadSamples(string fileName)
        {
            string[] lines = System.IO.File.ReadAllLines(fileName);
            _samples.Clear();
            foreach (string line in lines)
            {
                if (line.Length == 0)
                    continue;
                string newline = RemoveMultipleSpaces(line.Trim());
                string[] fields = newline.Split(' ');
                if (fields.Count() < 13)
                    return false;
                SP7Controller.SP7Pose x = new SP7Controller.SP7Pose();
                SP7Controller.SP7Vel xdot = new SP7Controller.SP7Vel();
                double time;
                double zulu_time;
                if (!Double.TryParse(fields[0], NumberStyles.Any, CultureInfo.InvariantCulture, out time))
                    return false;
                if (!Double.TryParse(fields[1], NumberStyles.Any, CultureInfo.InvariantCulture, out x.x))
                    return false;
                if (!Double.TryParse(fields[2], NumberStyles.Any, CultureInfo.InvariantCulture, out x.y))
                    return false;
                if (!Double.TryParse(fields[3], NumberStyles.Any, CultureInfo.InvariantCulture, out x.z))
                    return false;
                if (!Double.TryParse(fields[4], NumberStyles.Any, CultureInfo.InvariantCulture, out x.roll))
                    return false;
                if (!Double.TryParse(fields[5], NumberStyles.Any, CultureInfo.InvariantCulture, out x.pitch))
                    return false;
                if (!Double.TryParse(fields[6], NumberStyles.Any, CultureInfo.InvariantCulture, out x.yaw))
                    return false;
                if (!Double.TryParse(fields[7], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vx))
                    return false;
                if (!Double.TryParse(fields[8], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vy))
                    return false;
                if (!Double.TryParse(fields[9], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vz))
                    return false;
                if (!Double.TryParse(fields[10], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vroll))
                    return false;
                if (!Double.TryParse(fields[11], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vpitch))
                    return false;
                if (!Double.TryParse(fields[12], NumberStyles.Any, CultureInfo.InvariantCulture, out xdot.vyaw))
                    return false;
                if (fields.Count() >= 14)
                {
                    if (!Double.TryParse(fields[13], NumberStyles.Any, CultureInfo.InvariantCulture, out zulu_time))
                        return false;
                } else {
                    zulu_time = 0;
                }
                Tuple<double, SP7Controller.SP7Pose, SP7Controller.SP7Vel, double> t = new Tuple<double, SP7Controller.SP7Pose, SP7Controller.SP7Vel, double>(time, x, xdot, zulu_time);
                _samples.Add(t);
            }
            return true;
        }

        private string RemoveMultipleSpaces(string v)
        {
            StringBuilder sb = new StringBuilder();
            int space_cnt = 0;
            foreach (char ch in v)
            {
                if ((ch == ' ') || (ch == '\t'))
                {
                    if (space_cnt == 0)
                        sb.Append(' ');
                    space_cnt++;
                }
                else
                {
                    sb.Append(ch);
                    space_cnt = 0;
                }
            }
            return sb.ToString();
        }

        void controllerFeeder(ControllerDefinition controller)
        {
            const double epsilon_lin = 0.001;
            const double epsilon_ang = 0.002;

            const double minx = -0.05;
            const double maxx = 0.05;
            const double miny = -0.05;
            const double maxy = 0.05;
            const double minz = SP7Controller.Z_ZERO - 0.10;
            const double maxz = SP7Controller.Z_ZERO + 0.05;
            const double minroll = -8.0 * Math.PI / 180.0;
            const double maxroll = 8.0 * Math.PI / 180.0;
            const double minpitch = -8.0 * Math.PI / 180.0;
            const double maxpitch = 8.0 * Math.PI / 180.0;
            const double minyaw = -Math.PI;
            const double maxyaw = Math.PI;

            if (joystick == null)
                return;

            feeding = true;
            UpdateControls();
            joystick.Properties.AxisMode = DeviceAxisMode.Relative;
            joystick.Properties.BufferSize = 128;
            joystick.Acquire();

            SP7Controller.SP7Pose x;
            SP7Controller.SP7Vel xdot;

            double last_t = 0.0;
            x.x = x.y = 0.0;
            x.z = SP7Controller.Z_ZERO;
            x.roll = x.pitch = x.yaw = 0.0;
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;


            Stopwatch stopwatch = Stopwatch.StartNew();
            UInt32 buttons = 0;

            Dispatcher.Invoke(() =>
            {
                ControllerX.Minimum = 0;
                ControllerX.Maximum = controller.Saturation;
                ControllerY.Minimum = 0;
                ControllerY.Maximum = controller.Saturation;
                ControllerZ.Minimum = 0;
                ControllerZ.Maximum = controller.Saturation;
                ControllerRoll.Minimum = 0;
                ControllerRoll.Maximum = controller.Saturation;
                ControllerPitch.Minimum = 0;
                ControllerPitch.Maximum = controller.Saturation;
                ControllerYaw.Minimum = 0;
                ControllerYaw.Maximum = controller.Saturation;
            });

            bool lockRotations = false;
            bool lastLRPressed = false;
            bool lockTranslations = false;
            bool lastLTPressed = false;

            while (feeding && _sp7.IsConnected)
            {
                long ms = stopwatch.ElapsedMilliseconds;
                double t = ((double)ms) / 1000.0;
                double dt = t - last_t;
                joystick.Poll();
                var data = joystick.GetBufferedData();

                xdot.vx = xdot.vy = xdot.vz = 0.0;
                xdot.vroll = xdot.vpitch = xdot.vyaw = 0.0;
                foreach (var state in data)
                {
                    System.Console.WriteLine($"{state.Offset} {state.Value}");
                    if ((state.Offset >= JoystickOffset.Buttons0) && (state.Offset <= JoystickOffset.Buttons31))
                    {
                        if (state.Value != 0)
                            buttons |= 1U << (state.Offset - JoystickOffset.Buttons0);
                        else
                            buttons &= ~(1U << (state.Offset - JoystickOffset.Buttons0));
                        continue;
                    }

                    for (int i = 0; i < controller.Axis.Count(); i++)
                    {
                        if (state.Offset == controller.Axis[i])
                        {
                            if ((buttons & controller.AxisButtonMask[i]) == controller.AxisButtonState[i])
                            {
                                int v = (controller.AxisInvert[i]) ? -state.Value : state.Value;
                                if ((state.Value >= 0 ? state.Value : -state.Value) < (controller.Saturation * controller.DeadZone / 100))
                                    v = 0;
                                if (v > controller.Saturation)
                                    v = controller.Saturation;
                                if (v < -controller.Saturation)
                                    v = -controller.Saturation;
                                double vlin = (double)v / (double)controller.Saturation * 0.05 * dt;
                                double vang = (double)v / (double)controller.Saturation * 0.20 * dt;
                                ProgressBar ctrl = null;
                                switch (i)
                                {
                                    case 0:
                                        if (!lockTranslations)
                                        {
                                            ControllerAxis(ref x.x, minx, maxx, epsilon_lin, vlin, dt, ref xdot.vx);
                                            ctrl = ControllerX;
                                        }
                                        break;

                                    case 1:
                                        if (!lockTranslations)
                                        {
                                            ControllerAxis(ref x.y, miny, maxy, epsilon_lin, vlin, dt, ref xdot.vy);
                                            ctrl = ControllerY;
                                        }
                                        break;

                                    case 2:
                                        if (!lockTranslations)
                                        {
                                            ControllerAxis(ref x.z, minz, maxz, epsilon_lin, vlin, dt, ref xdot.vz);
                                            ctrl = ControllerZ;
                                        }
                                        break;

                                    case 3:
                                        if (!lockRotations)
                                        {
                                            ControllerAxis(ref x.roll, minroll, maxroll, epsilon_ang, vang, dt, ref xdot.vroll);
                                            ctrl = ControllerRoll;
                                        }
                                        break;

                                    case 4:
                                        if (!lockRotations)
                                        {
                                            ControllerAxis(ref x.pitch, minpitch, maxpitch, epsilon_ang, vang, dt, ref xdot.vpitch);
                                            ctrl = ControllerPitch;
                                        }
                                        break;

                                    case 5:
                                        if (!lockRotations)
                                        {
                                            ControllerAxis(ref x.yaw, minyaw, maxyaw, epsilon_ang, vang * 4, dt, ref xdot.vyaw);
                                            ctrl = ControllerYaw;
                                        }
                                        break;
                                }

                                if (ctrl != null)
                                {
                                    Dispatcher.Invoke(() =>
                                    {
                                        ctrl.Value = v > 0 ? v : -v;
                                    });
                                }
                            }
                            break;
                        }
                    }
                }

                if ((controller.ZeroButtonMask != 0) && ((buttons & controller.ZeroButtonMask) == controller.ZeroButtonMask))
                {
                    x.x = x.y = 0.0;
                    x.z = SP7Controller.Z_ZERO;
                    x.roll = x.pitch = x.yaw = 0.0;
                }

                if (controller.LockRButtonMask != 0)
                {
                    if ((buttons & controller.LockRButtonMask) == controller.LockRButtonMask)
                    {
                        if (!lastLRPressed)
                        {
                            lockRotations = !lockRotations;
                            Dispatcher.Invoke(() =>
                            {
                                ControllerYaw.Visibility = lockRotations ? Visibility.Hidden : Visibility.Visible;
                                ControllerPitch.Visibility = lockRotations ? Visibility.Hidden : Visibility.Visible;
                                ControllerRoll.Visibility = lockRotations ? Visibility.Hidden : Visibility.Visible;
                            });
                        }
                        lastLRPressed = true;
                    }
                    else
                    {
                        lastLRPressed = false;
                    }
                }

                if (controller.LockTButtonMask != 0)
                {
                    if ((buttons & controller.LockTButtonMask) == controller.LockTButtonMask)
                    {
                        if (!lastLTPressed)
                        {
                            lockTranslations = !lockTranslations;
                            Dispatcher.Invoke(() =>
                            {
                                ControllerX.Visibility = lockTranslations ? Visibility.Hidden : Visibility.Visible;
                                ControllerY.Visibility = lockTranslations ? Visibility.Hidden : Visibility.Visible;
                                ControllerZ.Visibility = lockTranslations ? Visibility.Hidden : Visibility.Visible;
                            });
                        }
                        lastLTPressed = true;
                    }
                    else
                    {
                        lastLTPressed = false;
                    }
                }

                _sp7.SetEETargets(x, xdot);
                UpdateTargetPose(x, xdot);
                last_t = t;

                try
                {
                    Update3D(x);
                } catch (Exception ex) { }

                Thread.Sleep(1000 / 60);
            }
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);
            stopwatch.Stop();
            joystick.Unacquire();
            Thread.Sleep(100);
            _sp7.Reset();
            feeding = false;
            UpdateControls();
        }

        private void ControllerAxis(ref double pos, double min, double max, double epsilon, double val, double dt, ref double vel)
        {
            if (((pos - max) > epsilon) && (val > 0))
            {
                pos = max;
                vel = 0;
            }
            else if (((pos - min) < epsilon) && (val < 0))
            {
                pos = min;
                vel = 0;
            }
            else
            {
                pos += val;
                vel = val / dt;
                if (pos > max)
                {
                    pos = max;
                    vel = 0;
                }
                if (pos < min)
                {
                    pos = min;
                    vel = 0;
                }
            }
        }


        void simFeeder(UInt16 port)
        {
            const float maxangle = 8.0f; 
            UdpClient channel = new UdpClient(port);
            IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
            feeding = true;
            UpdateControls();

            SP7Controller.SP7Pose x;
            SP7Controller.SP7Vel xdot;
            x.x = x.y = x.roll = x.pitch = x.yaw = 0;
            x.z = SP7Controller.Z_ZERO;
            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;

            while (feeding && _sp7.IsConnected)
            {
                var asyncResult = channel.BeginReceive(null, null);
                asyncResult.AsyncWaitHandle.WaitOne(100);

                if (asyncResult.IsCompleted)
                {
                    byte[] simdata = channel.EndReceive(asyncResult, ref remote);
                    MemoryStream ms = new MemoryStream(simdata);
                    BinaryReader mr = new BinaryReader(ms);
                    float t = mr.ReadSingle();
                    float ax = mr.ReadSingle();
                    float ay = mr.ReadSingle();
                    float az = mr.ReadSingle();
                    float pitch = mr.ReadSingle();
                    float yaw = mr.ReadSingle();
                    float roll = mr.ReadSingle();
                    float vpitch = mr.ReadSingle();
                    float vyaw = mr.ReadSingle();
                    float vroll = mr.ReadSingle();
                    float GS = mr.ReadSingle();
                    float mass = mr.ReadSingle();

                    yaw = -((yaw % 360.0f) - 180.0f); // compass to platform
                
                    if (pitch > maxangle) { pitch = maxangle; vpitch = 0.0f; }
                    if (pitch < -maxangle) { pitch = -maxangle; vpitch = 0.0f; }
                    if (roll > maxangle) { roll = maxangle; vroll = 0.0f; }
                    if (roll < -maxangle) { roll = -maxangle; vroll = 0.0f; }
                    if (yaw > 180.0) { yaw = 180.0f; vyaw = 0.0f; }
                    if (yaw < -180.0) { yaw = -180.0f; vyaw = 0.0f; }

                    x.roll = roll * Math.PI / 180.0;
                    x.pitch = pitch * Math.PI / 180.0;
                    x.yaw = yaw * Math.PI / 180.0;
//                    xdot.vroll = vroll * Math.PI / 180.0;
//                    xdot.vpitch = vpitch * Math.PI / 180.0;
//                    xdot.vyaw = vyaw * Math.PI / 180.0;

                    try
                    {
                        Update3D(x);
                        Dispatcher.Invoke(() =>
                        {
                            SimAX.Text = $"{ax:#.000}";
                            SimAY.Text = $"{ay:#.000}";
                            SimAZ.Text = $"{az:#.000}";
                            SimVRoll.Text = $"{roll:#.000}";
                            SimVPitch.Text = $"{pitch:#.000}";
                            SimVYaw.Text = $"{yaw:#.000}";
                        });
                    }
                    catch (Exception ex) { }
                }

                _sp7.SetEETargets(x, xdot);
                UpdateTargetPose(x, xdot);
            }

            xdot.vx = xdot.vy = xdot.vz = xdot.vroll = xdot.vpitch = xdot.vyaw = 0;
            _sp7.SetEETargets(x, xdot);

            Thread.Sleep(100);
            _sp7.Reset();
            channel.Close();
            feeding = false;
            UpdateControls();
        }

        static Quaternion QFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            float halfRoll = -roll * 0.5f;
            float sinHalfRoll = (float)Math.Sin((double)halfRoll);
            float cosHalfRoll = (float)Math.Cos((double)halfRoll);
            float halfPitch = -pitch * 0.5f;
            float sinHalfPitch = (float)Math.Sin((double)halfPitch);
            float cosHalfPitch = (float)Math.Cos((double)halfPitch);
            float halfYaw = -yaw * 0.5f;
            float sinHalfYaw = (float)Math.Sin((double)halfYaw);
            float cosHalfYaw = (float)Math.Cos((double)halfYaw);
            Quaternion result = new Quaternion();
            result.W = cosHalfPitch * cosHalfRoll * cosHalfYaw + sinHalfPitch * sinHalfRoll * sinHalfYaw;
            result.X = sinHalfPitch * cosHalfRoll * cosHalfYaw + cosHalfPitch * sinHalfRoll * sinHalfYaw;
            result.Y = cosHalfPitch * sinHalfRoll * cosHalfYaw + sinHalfPitch * cosHalfRoll * sinHalfYaw;
            result.Z = cosHalfPitch * cosHalfRoll * sinHalfYaw + sinHalfPitch * sinHalfRoll * cosHalfYaw;
            return result;
        }



    }

    }
