using AGV_V1._0.Agv;
using AGV_V1._0.Algorithm;
using AGV_V1._0.DataBase;
using AGV_V1._0.Event;
using AGV_V1._0.Network.ThreadCode;
using AGV_V1._0.NLog;
using AGV_V1._0.Queue;
using AGV_V1._0.Server.APM;
using AGV_V1._0.Util;
using AGVSocket.Network;
using AGVSocket.Network.EnumType;
using AGVSocket.Network.Packet;
using Agv.PathPlanning;
using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AGV_V1._0
{
    class VehicleManager : BaseThread
    {
        //新建两个全局对象  小车
        private static Vehicle[] vehicles;
        List<Vehicle> vFinished = new List<Vehicle>();
        private bool vehicleInited = false;
        private double moveCount = 0;//统计移动的格数，当前地图一格1.5米
        public const int REINIT_COUNT = 20;
        private const int WAIT_TIME = 8;//  等待超时后还没有翻盘完成的消息就重发翻盘报文

        private static Random rand = new Random(1);//5,/4/4 //((int)DateTime.Now.Ticks);//随机数，随机产生坐标


        private static VehicleManager instance;
        public static VehicleManager Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new VehicleManager();
                }
                return instance;
            }
        }
        private VehicleManager()
        {

        }
        protected override string ThreadName()
        {
            return "VehicleManager";
        }
        byte serinum = 1;
        protected override void Run()
        {
            Thread.Sleep(ConstDefine.STEP_TIME);

            if (vehicles == null)
            {
                return;
            }
            for (int vnum = 0; vnum < vehicles.Length; vnum++)
            {
                serinum = (byte)(serinum % 255);
                if (vehicles[vnum].CurState == State.cannotToDestination && vehicles[vnum].Arrive == false)
                {
                    vehicles[vnum].Arrive = true;
                    vFinished.Add(vehicles[vnum]);
                    vehicles[vnum].Route.Clear();
                    string str = string.Format("小车" + vnum + ":({0}，{1})->({2}，{3})没有搜索到路径，", vehicles[vnum].BeginX, vehicles[vnum].BeginY, vehicles[vnum].EndX, vehicles[vnum].EndY);
                    OnShowMessage(this, new MessageEventArgs(str));
                    continue;
                }
                if (vehicles[vnum].Route == null || vehicles[vnum].Route.Count <1)
                {
                    continue;
                }
                if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.carried)
                {
                    if (vehicles[vnum].EqualWithRealLocation(vehicles[vnum].BeginX, vehicles[vnum].BeginY))
                    {
                        if (vehicles[vnum].agvInfo.AgvMotion == AgvMotionState.StopedNode)
                        {
                            TrayPacket tp = new TrayPacket(serinum++, 4, TrayMotion.TopLeft);
                           // SendPacketQueue.Instance.Enqueue(tp);
                            AgvServerManager.Instance.Send(tp);
                            vehicles[vnum].CurState = State.unloading;
                            vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME);
                            Console.WriteLine("send TrayMotion:"+(serinum-1));
                        }
                    }
                    //else
                    //{
                    //    uint x = Convert.ToUInt32(vehicles[vnum].BeginX);
                    //    uint y = Convert.ToUInt32(vehicles[vnum].BeginY);
                    //    uint endX = Convert.ToUInt32(vehicles[vnum].EndX);
                    //    uint endY = Convert.ToUInt32(vehicles[vnum].EndY);
                    //    RunPacket rp = new RunPacket(serinum, 4, MoveDirection.Forward, 1500, new Destination(new CellPoint(x * ConstDefine.CELL_UNIT, y * ConstDefine.CELL_UNIT), new CellPoint(endX * ConstDefine.CELL_UNIT, endY * ConstDefine.CELL_UNIT), new AgvDriftAngle(0), TrayMotion.None));
                    //    //asm.Send(rp);
                    //    SendPacketQueue.Instance.Enqueue(rp);

                    //    Console.WriteLine("unloaing resend");
                    //}
                    continue;
                }                
                if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.unloading)
                {
                    if (DateTime.Now > vehicles[vnum].WaitEndTime)
                    {
                        if (vehicles[vnum].EqualWithRealLocation(vehicles[vnum].BeginX, vehicles[vnum].BeginY))
                        {
                            if (vehicles[vnum].agvInfo.AgvMotion == AgvMotionState.StopedNode)
                            {
                                TrayPacket tp = new TrayPacket(serinum++, 4, TrayMotion.TopLeft);
                                // SendPacketQueue.Instance.Enqueue(tp);
                                AgvServerManager.Instance.Send(tp);
                                vehicles[vnum].CurState = State.unloading;
                                vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME);
                                Console.WriteLine("resend TrayMotion**********:" + (serinum - 1));
                            }
                        }
                    }
                    continue;
                }
                if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.Free)
                {                    
                    vFinished.Add(vehicles[vnum]);
                    vehicles[vnum].Route.Clear();
                    vehicles[vnum].LockNode.Clear();

                    Console.WriteLine("下一个目标：");
                    RandomMove(4);
                    continue;
                }               
                if (vehicles[vnum].StopTime < 0)
                {
                    if (vehicles[vnum].CurNodeTypy() != MapNodeType.queuingArea && GetDirCount(vehicles[vnum].BeginX, vehicles[vnum].BeginY) > 1)
                    {
                        if (vehicles[vnum].Stoped > -1 && vehicles[vnum].Stoped < vehicles.Length)
                        {
                            vehicles[vehicles[vnum].Stoped].StopTime = 2;
                        }
                        //重新搜索路径
                        SearchRoute(vnum, true);
                    }
                    vehicles[vnum].StopTime = 3;
                }
                else
                {
                    vehicles[vnum].Move(ElecMap.Instance);
                    bool isMove = false;// vehicles[vnum].Move(ElecMap.Instance);
                        if (isMove)
                        {
                            uint x = Convert.ToUInt32(vehicles[vnum].BeginX);
                            uint y = Convert.ToUInt32(vehicles[vnum].BeginY);
                            uint endX = Convert.ToUInt32(vehicles[vnum].EndX);
                            uint endY = Convert.ToUInt32(vehicles[vnum].EndY);                            
                            RunPacket rp = new RunPacket(serinum++, 4, MoveDirection.Forward, 1500, new Destination(new CellPoint(x * ConstDefine.CELL_UNIT, y * ConstDefine.CELL_UNIT), new CellPoint(endX * ConstDefine.CELL_UNIT, endY * ConstDefine.CELL_UNIT), new AgvDriftAngle(0), TrayMotion.None));
                            AgvServerManager.Instance.Send(rp);

                            Console.WriteLine("*--------------------------------------------------------------------------*");
                            Console.WriteLine(vehicles[vnum].TPtr+":"+x + "," + y + "->" + endX + "," + endY + " ,实际位置：" + vehicles[vnum].agvInfo.CurLocation.CurNode.X / 1000.0 + "," + vehicles[vnum].agvInfo.CurLocation.CurNode.Y / 1000.0+"序列号："+(serinum-1));

                            CheckAlarmState(vnum);
                            vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME);

                            moveCount++;
                            OnShowMessage(string.Format("{0:N} 公里", (moveCount * 1.5) / 1000.0));
                        }

                }

                


            }
            if (vFinished != null)
            {
                for (int i = 0; i < vFinished.Count; i++)
                {
                    FinishedQueue.Instance.Enqueue(vFinished[i]);
                }
                vFinished.Clear();
            }

        }
        void CheckAlarmState(int vnum)
        {
            if (vehicles[vnum].agvInfo.Alarm == AlarmState.ScanNone || vehicles[vnum].agvInfo.Alarm == AlarmState.CommunicationFault)
            {
                Logs.Error("通信故障或没扫到码");
                MessageBox.Show("通信故障或没扫到码");
            }
        }
        
       
        int GetDirCount(int row, int col)
        {
            int dir = 0;
            if (ElecMap.Instance.mapnode[row, col].RightDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].LeftDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].DownDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].UpDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            return dir;
        }
        /// <summary>
        /// 初始化小车
        /// </summary>
        public void InitialVehicle()
        {
            vehicleInited = false;
            //初始化小车位置

            if (null == FileUtil.sendData || FileUtil.sendData.Length < 1)
            {
                throw new ArgumentNullException();
            }
            int vehicleCount = FileUtil.sendData.Length;
            vehicles = new Vehicle[vehicleCount];
            for (int i = 0; i < vehicleCount; i++)
            {
                vehicles[i] = new Vehicle(FileUtil.sendData[i].BeginX, FileUtil.sendData[i].BeginY, i, false, Direction.Right);
                //MyPoint endPoint = RouteUtil.RandPoint(ElecMap.Instance);
                //MyPoint mp = SqlManager.Instance.GetVehicleCurLocationWithId(i);
                //if (mp != null)
                //{
                //    vehicles[i].BeginX = mp.X;
                //    vehicles[i].BeginY = mp.Y;
                //}
                int R = rand.Next(20, 225);
                int G = rand.Next(20, 225);
                int B = rand.Next(20, 225);
                vehicles[i].pathColor = Color.FromArgb(80, R, G, B);
                vehicles[i].showColor = Color.FromArgb(255, R, G, B);
            }

            vehicleInited = true;
            ////把小车所在的节点设为占用状态
            RouteUtil.VehicleOcuppyNode(ElecMap.Instance, vehicles);

        }
        public void ReInitWithiRealAgv()
        {
            
            bool res = false;
            int count=1;
            while (res == false && count < REINIT_COUNT)
            {
                Thread.Sleep(count * 50);
                for (int i = 0; i < vehicles.Length; i++)
                {
                    if (vehicles[i].agvInfo != null)
                    {
                        vehicles[i].BeginX = (int)Math.Round(vehicles[i].agvInfo.CurLocation.CurNode.X/1000.0);
                        vehicles[i].BeginY = (int)Math.Round(vehicles[i].agvInfo.CurLocation.CurNode.Y/1000.0);
                        res = true;
                    }
                }
                count++;
            }
            if (count >= REINIT_COUNT)
            {
                MessageBox.Show("没有小车连接，请检查ip设置是否有问题");
            }
        }
        public void AddOrUpdate(ushort agvId, AgvInfo info)
        {
            if (vehicleInited == false)
            {
                return;
            }
            if (agvId >= vehicles.Length)
            {
                Logs.Error("程序预设的agv数量少了");
                return;
            }
            if (info == null)
            {
                return;
            }
            vehicles[(int)agvId].agvInfo = info;
        }
        public void RandomMove(int Id)
        {           
            
            MyPoint mpEnd = RouteUtil.RandRealPoint(ElecMap.Instance);
            while (mpEnd.X == vehicles[Id].BeginX && mpEnd.Y == vehicles[Id].BeginY)
            {
                mpEnd = RouteUtil.RandRealPoint(ElecMap.Instance);
            }
            SendData sd = new SendData(Id, vehicles[Id].BeginX, vehicles[Id].BeginY, mpEnd.X, mpEnd.Y);
            sd.Arrive = false;
            sd.EndLoc = "rest";
            sd.State = State.carried;

            SearchRouteQueue.Instance.Enqueue(new SearchData(sd, false));


        }
        void SearchRoute(int num, bool isResarch)
        {
            SendData td = new SendData();
            td.Num = num;
            td.BeginX = vehicles[num].BeginX;
            td.BeginY = vehicles[num].BeginY;
            td.EndX = vehicles[num].EndX;
            td.EndY = vehicles[num].EndY;
            td.Arrive = false;
            td.EndLoc = vehicles[num].EndLoc;
            td.StartLoc = vehicles[num].StartLoc;
            td.State = vehicles[num].CurState;


            //if (!ElecMap.Instance.IsSpecialArea(td.BeginX, td.BeginY) && ElecMap.Instance.IsScanner(td.EndX, td.EndY))
            //{
            //    MessageBox.Show("起点：" + td.BeginX + "," + td.BeginY + "" + "终点：" + td.EndX + "," + td.EndY);
            //}
            SearchRouteQueue.Instance.Enqueue(new SearchData(td, isResarch));

            //Task.Factory.StartNew(() => vehicle[num].SearchRoute(Elc), TaskCreationOptions.LongRunning);
        }

        private static readonly object vehicleLock = new object();
        public Vehicle[] GetVehicles()
        {
            //Vehicle[] v = null;
            //lock (vehicleLock)
            //{
            //    if (vehicles != null)
            //    {
            //        v = new Vehicle[ConstDefine.g_VehicleCount];
            //        for (int i = 0; i < vehicles.Length; i++)
            //        {
            //            v[i] = vehicles[i].CloneDeep();
            //        }
            //    }
            //}
            //return v;
            return vehicles;
        }


    }

}
