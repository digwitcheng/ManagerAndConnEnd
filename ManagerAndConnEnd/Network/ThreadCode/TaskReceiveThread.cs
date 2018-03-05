using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using AGV_V1._0.Queue;
using AGV_V1._0.Util;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.ThreadCode
{
    class TaskReceiveThread:BaseThread
    {
        
        private static TaskReceiveThread instance;
        public static TaskReceiveThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new TaskReceiveThread();
                }
                return instance;
            }
        }
        private TaskReceiveThread()
        {
            
        }
        protected override string ThreadName()
        {
            return "TaskReceive";
        }
        protected override void Run()
        {
            try
            {
                if (TaskRecvQueue.Instance.IsHasData())
                {
                    string json = TaskRecvQueue.Instance.Dequeue();
                    if (json == null)
                    {
                        Logs.Warn("没取到任务");
                        OnShowMessage(this, new MessageEventArgs("没取到任务"));
                        return;
                    }
                    if (json.Trim() == "")
                    {
                        Logs.Warn("收到空任务");
                        OnShowMessage(this, new MessageEventArgs("收到空任务"));
                        return;
                    }
                    try
                    {
                        SendData sd = (SendData)JsonConvert.DeserializeObject(json, typeof(SendData));
                        if (sd != null)
                        {
                            SearchRouteQueue.Instance.Enqueue(new SearchData(sd, false));
                        }
                    }
                    catch (Exception ex)
                    {
                        Logs.Error("把任务加到搜索队列出错:" + ex.ToString());
                        OnShowMessage(this, new MessageEventArgs("把任务加到搜索队列出错:" + ex.ToString()));
                    }
                }
                Thread.Sleep(ConstDefine.TASK_TIME);
            }
            catch (Exception ex)
            {
                Logs.Error("taskreceive出错");
            }
        }
    }
}
