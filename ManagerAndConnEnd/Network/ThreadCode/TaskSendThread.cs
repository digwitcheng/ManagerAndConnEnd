using AGV_V1._0.Agv;
using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using AGV_V1._0.Queue;
using AGV_V1._0.Server.APM;
using AGV_V1._0.Util;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.ThreadCode
{
    class TaskSendThread:BaseThread
    {
        private int finishCount = 0;
        private int perCount = 0;//一定时间内的数量
        private int per = 0;//时间间隔计数器


         private static TaskSendThread instance;
        public static TaskSendThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new TaskSendThread();
                }
                return instance;
            }
        }
        private TaskSendThread()
        {            
        }
        protected override string ThreadName()
        {
            return "TaskSend";
        }
        DateTime startTime = DateTime.Now.AddMinutes(10);
        protected override void Run()
        {
            try
            {                
                if (FinishedQueue.Instance.IsHasData())
                {
                    Vehicle v = FinishedQueue.Instance.Dequeue();
                    if (v.CurState == State.unloading)
                    {
                        DateTime nowTime = DateTime.Now;
                        if (nowTime > startTime)
                        {
                            per++;
                            Logs.Info(nowTime.ToString()+" "+per + ":" + perCount);
                            startTime = DateTime.Now.AddMinutes(10);
                            perCount = 0;
                            
                        }
                        perCount++;
                        finishCount++;
                        OnShowMessage(finishCount + "");
                        Unloading(v);

                    }
                    else
                    {
                        string json = JsonHelper.VehicleToJson(v);
                        // TaskLisenter.Instance.SendVehicleData(json);                    
                        TaskServerManager.Instance.Send(MessageType.Arrived, json);
                    }
                }
                Thread.Sleep(ConstDefine.TASK_TIME);
            }
            catch { }
        }
        void Unloading(Vehicle v)
        {
            Task.Factory.StartNew(() =>
            {
                System.Threading.Thread.Sleep(ConstDefine.UNLOADING_TIME);
                v.CurState = State.Free;
                FinishedQueue.Instance.Enqueue(v);
            });

        }
    }
}
