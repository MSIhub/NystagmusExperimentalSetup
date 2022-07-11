using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Fune
{
    public class SP7Controller
    {
        public event MotionDataEventHandler MotionDataReceived;
        public delegate void MotionDataEventHandler(object sender, MotionData md);

        public event ConnectionStateEventHandler ConnectionStateChanged;
        public delegate void ConnectionStateEventHandler(object sender, bool connected);

        bool _simulate = false;
        public bool Simulate { get { return _simulate; } set { _simulate = value; } }
        public bool IsConnected {  get { return _connected || Simulate; } }

        const Int32 port = 0x4654;
        const UInt32 magic = 0x41545353;
        const int packet_len = 190;

        IPEndPoint _ipe;
        UdpClient _channel;
        UInt32 pid = 0;
        Thread _rxThread;
        bool _stopRx = false;
        Dictionary<UInt32, Waiter> _waiters = new Dictionary<UInt32, Waiter>();
        bool _connected;

        public const double Z_ZERO = 0.401;

        enum CtrlCommand
        {
            ALIVE = 0,
            CONNECT,
            DISCONNECT,
            START,
            STOP,
            ZERO,
            JOINT_TARGETS,
            EE_TARGETS,
            RESET = 0xeffe,
            HALT = 0xefff,
            MOTION_DATA = 0xfffe,
            CMD_REPLY = 0xffff
        }

        public struct SP7Pose
        {
            public double x, y, z, roll, pitch, yaw;
        }

        public struct SP7Vel
        {
            public double vx, vy, vz, vroll, vpitch, vyaw;
        }

        public struct SP7JointPos
        {
            public double ax1, ax2, ax3, ax4, ax5, ax6, ax7;
        }

        public struct SP7JointVel
        {
            public double vax1, vax2, vax3, vax4, vax5, vax6, vax7;
        }


        public SP7Controller(string address, bool simulate = false)
        {
            _ipe = new IPEndPoint(IPAddress.Parse(address), port);
            _channel = new UdpClient();
            _channel.Connect(_ipe);
            _connected = false;
            Simulate = simulate;
        }


        public bool Connect()
        {
            _stopRx = false;
            _rxThread = new Thread(RxWorker);
            _rxThread.Start();
            if (!Simulate)
            {
                _connected = SendCommand(CtrlCommand.CONNECT, new byte[0]);

            }
            if (IsConnected && (ConnectionStateChanged != null))
                ConnectionStateChanged(this, true);
            return IsConnected;
        }


        public bool Disconnect()
        {
            _stopRx = true;
            if (!_rxThread.Join(1000))
                _rxThread.Abort();
            bool res = true;
            if (!Simulate)
            {
                res = SendCommand(CtrlCommand.DISCONNECT, new byte[0]);
                _connected = false;
            }
            if (ConnectionStateChanged != null)
                ConnectionStateChanged(this, false);
            return res;
        }


        private void AliveFail()
        {
            _stopRx = true;
            _connected = false;
            if (ConnectionStateChanged != null)
                ConnectionStateChanged(this, false);
        }


        internal bool SetEETargets(SP7Pose x, SP7Vel xdot)
        {
            if (Simulate)
                return true;
            MemoryStream ms = new MemoryStream();
            BinaryWriter bw = new BinaryWriter(ms);
            bw.Write((Int32)(x.x * (1 << 16)));
            bw.Write((Int32)(x.y * (1 << 16)));
            bw.Write((Int32)(x.z * (1 << 16)));
            bw.Write((Int32)(x.roll * (1 << 16)));
            bw.Write((Int32)(x.pitch * (1 << 16)));
            bw.Write((Int32)(x.yaw * (1 << 16)));
            bw.Write((Int32)(xdot.vx * (1 << 16)));
            bw.Write((Int32)(xdot.vy * (1 << 16)));
            bw.Write((Int32)(xdot.vz * (1 << 16)));
            bw.Write((Int32)(xdot.vroll * (1 << 16)));
            bw.Write((Int32)(xdot.vpitch * (1 << 16)));
            bw.Write((Int32)(xdot.vyaw * (1 << 16)));
            byte[] data = ms.ToArray();
            return SendCommand(CtrlCommand.EE_TARGETS, data, false);
        }

        internal bool SetJointTargets(SP7JointPos j, SP7JointVel jdot)
        {
            if (Simulate)
                return true;
            MemoryStream ms = new MemoryStream();
            BinaryWriter bw = new BinaryWriter(ms);
            bw.Write((Int32)(j.ax1 * (1 << 16)));
            bw.Write((Int32)(j.ax2 * (1 << 16)));
            bw.Write((Int32)(j.ax3 * (1 << 16)));
            bw.Write((Int32)(j.ax4 * (1 << 16)));
            bw.Write((Int32)(j.ax5 * (1 << 16)));
            bw.Write((Int32)(j.ax6 * (1 << 16)));
            bw.Write((Int32)(j.ax7 * (1 << 16)));
            bw.Write((Int32)(jdot.vax1 * (1 << 16)));
            bw.Write((Int32)(jdot.vax2 * (1 << 16)));
            bw.Write((Int32)(jdot.vax3 * (1 << 16)));
            bw.Write((Int32)(jdot.vax4 * (1 << 16)));
            bw.Write((Int32)(jdot.vax5 * (1 << 16)));
            bw.Write((Int32)(jdot.vax6 * (1 << 16)));
            bw.Write((Int32)(jdot.vax7 * (1 << 16)));
            byte[] data = ms.ToArray();
            return SendCommand(CtrlCommand.JOINT_TARGETS, data, false);
        }


        public bool Reset()
        {
            if (Simulate)
                return true;
            return SendCommand(CtrlCommand.RESET, new byte[0]);
        }


        public bool Halt()
        {
            if (Simulate)
                return true;
            return SendCommand(CtrlCommand.HALT, new byte[0]);
        }


        void SendAlive()
        {
            if (Simulate)
                return;

            UInt32 id;
            lock (_channel)
            {
                id = pid++;
            }

            MemoryStream ms = new MemoryStream();
            BinaryWriter bw = new BinaryWriter(ms);
            bw.Write(magic);
            bw.Write(id);
            bw.Write((UInt16)CtrlCommand.ALIVE);
            int delta = packet_len - 10;
            while (delta > 0)
            {
                bw.Write((byte)0);
                delta--;
            }
            byte[] buf = ms.ToArray();
            _channel.Send(buf, buf.Length);
        }


        bool SendCommand(CtrlCommand cmd, byte[] data, bool has_reply = true)
        {
            UInt32 id;
            lock (_channel)
            {
                id = pid++;
            }

            MemoryStream ms = new MemoryStream();
            BinaryWriter bw = new BinaryWriter(ms);
            bw.Write(magic);
            bw.Write(id);
            bw.Write((UInt16)cmd);
            bw.Write(data);
            int delta = packet_len - (data.Count() + 10);
            while (delta > 0)
            {
                bw.Write((byte)0);
                delta--;
            }
            byte[] buf = ms.ToArray();
            if (_channel.Send(buf, buf.Length) == ms.Length)
            {
                if (has_reply)
                {
                    bool res;
                    Waiter wo = new Waiter(id, cmd);
                    lock (_waiters)
                    {
                        _waiters[id] = wo;
                    }
                    lock (wo)
                    {
                        res = Monitor.Wait(wo, 100);
                    }
                    lock (_waiters)
                    {
                        _waiters.Remove(id);
                    }
                    if (!res || (wo.Result != 0))
                        return false;
                    return true;
                } else
                {
                    return true;
                }
            }
            return false;
        }


        public void RxWorker()
        {
            int cnt = 0;
            int alive = 0;

            while (!_stopRx)
            {
                try
                {
                    var asyncResult = _channel.BeginReceive(null, null);
                    asyncResult.AsyncWaitHandle.WaitOne(100);

                    alive++;
                    if (alive >= 20)
                    {
                        AliveFail();
                        return;
                    }

                    if (asyncResult.IsCompleted)
                    {
                        byte[] reply = _channel.EndReceive(asyncResult, ref _ipe);
                        alive = 0;
                        MemoryStream ms = new MemoryStream(reply);
                        BinaryReader mr = new BinaryReader(ms);
                        if (mr.ReadUInt32() != magic)
                            continue;
                        UInt32 id = mr.ReadUInt32();
                        CtrlCommand cmd = (CtrlCommand)mr.ReadUInt16();

                        switch (cmd)
                        {
                            case CtrlCommand.CMD_REPLY:
                                cmd = (CtrlCommand)mr.ReadUInt16();
                                UInt32 res = mr.ReadUInt32();
                                if (_waiters.ContainsKey(id))
                                {
                                    Waiter wo;
                                    lock (_waiters)
                                    {
                                        wo = _waiters[id];
                                    }
                                    if ((wo != null) && (wo.Cmd == cmd))
                                    {
                                        wo.Result = res;
                                        lock (wo) Monitor.Pulse(wo);
                                    }
                                }
                                break;
                            case CtrlCommand.MOTION_DATA:
                                MotionData md = new MotionData(id, mr);
                                if (MotionDataReceived != null)
                                    MotionDataReceived(this, md);
                                break;
                        }
                        cnt++;
                        if (cnt >= 10)
                        {
                            SendAlive();
                            cnt = 0;
                        }
                    }
                } catch (Exception) {
                    Thread.Sleep(10);
                }
            }
        }


        class Waiter
        {
            public UInt32 Id { get; protected set; }
            public CtrlCommand Cmd { get; protected set; }
            public UInt32 Result { get; set; }

            public Waiter(UInt32 id, CtrlCommand cmd)
            {
                Id = id;
                Cmd = cmd;
            }
        }


        public class MotionData : EventArgs
        {
            public ControllerState SState { get; protected set; }
            public ControllerState[] DState { get; protected set; }
            public MotionState MState { get; protected set; }
            public double[] JointPosition { get; protected set; }
            public double[] JointVelocity { get; protected set; }
            public char IKState { get; protected set; }
            public char FKState { get; protected set; }
            public UInt32 Timestamp { get; protected set; }
            public double[] JointRefPosition { get; protected set; }
            public double[] JointRefVelocity { get; protected set; }
            public double[] PlatTargetPosition { get; protected set; }
            public double[] PlatTargetVelocity { get; protected set; }
            public enum MotionState
            {
                HALTED, INIT, IDLE, NEW_JOINT_TARGET, NEW_EE_TARGET, ZEROING
            }

            public enum ControllerState
            {
                COLD, INIT, RUNNING, HALTED
            }

            public MotionData(UInt32 ts, BinaryReader r)
            {
                Timestamp = ts;
                DState = new ControllerState[7];
                JointPosition = new double[7];
                JointVelocity = new double[7];
                JointRefPosition = new double[7];
                JointRefVelocity = new double[7];
                PlatTargetPosition = new double[6];
                PlatTargetVelocity = new double[6];

                SState = (ControllerState)r.ReadUInt16();
                MState = (MotionState)r.ReadUInt16();
                for (int joint = 0; joint < 7; joint++)
                    DState[joint] = (ControllerState)r.ReadUInt16();
                for (int joint = 0; joint < 7; joint++)
                    JointPosition[joint] = (double)r.ReadInt32() / (double)(1 << 16);
                for (int joint = 0; joint < 7; joint++)
                    JointVelocity[joint] = (double)r.ReadInt32() / (double)(1 << 16);

                IKState = r.ReadChar();
                FKState = r.ReadChar();

                for (int joint = 0; joint < 7; joint++)
                    JointRefPosition[joint] = (double)r.ReadInt32() / (double)(1 << 16);
                for (int joint = 0; joint < 7; joint++)
                    JointRefVelocity[joint] = (double)r.ReadInt32() / (double)(1 << 16);

                for (int joint = 0; joint < 6; joint++)
                    PlatTargetPosition[joint] = (double)r.ReadInt32() / (double)(1 << 16);
                for (int joint = 0; joint < 6; joint++)
                    PlatTargetVelocity[joint] = (double)r.ReadInt32() / (double)(1 << 16);
            }

        }


    }

}
