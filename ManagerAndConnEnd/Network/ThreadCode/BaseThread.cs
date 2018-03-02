using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.ThreadCode
{
    abstract class BaseThread
    {
         public event EventHandler<MessageEventArgs> ShowMessage;

         
         private bool isRunning = false;
         private static readonly object runLock = new object();
         bool IsRunning { get { return isRunning; } }
         
         public  void Start()
         {
             lock (runLock)
             {
                 if (!isRunning)
                 {
                     Task.Factory.StartNew(() => StartThread());//启动线程
                 }
             }
         }
          void StartThread()
         {
            // Logs.Info(ThreadName()+"线程开始");
             OnShowMessage(ThreadName()+"线程开始");
             isRunning = true;
             while (isRunning)
             {
                 try
                 {
                     Run();
                 }
                 catch (Exception ex)
                 {
                     Logs.Error(ThreadName() + "线程执行出错:" + ex);
                     OnShowMessage(ThreadName() + "线程执行出错:" + ex);
                 }
             }
             isRunning = false;
            // Logs.Info(ThreadName()+"线程结束");
             OnShowMessage(ThreadName()+"线程结束");
         }
         protected abstract void Run();

         string unnamed =""+DateTime.Now.Millisecond;
         protected virtual String ThreadName()
         {
             return unnamed;
         }
         public  void End()
         {
             lock (runLock)
             {
                 isRunning = false;
                 Thread.Sleep(2);
             }
         }
         protected void OnShowMessage(string str)
         {
             OnShowMessage(this, new MessageEventArgs(str));
         }
         protected void OnShowMessage(object sender, MessageEventArgs e)
         {
             if (null != ShowMessage)
             {
                 try
                 {
                     this.ShowMessage.Invoke(this, e);
                 }
                 catch
                 {
                 }
             }
         }
    }
}
