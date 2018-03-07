using AGV_V1._0.Event;
using AGV_V1._0.Network.Messages;
using AGV_V1._0.NLog;
using AGV_V1._0.Server.APM;
using AGV_V1._0.Util;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CowboyTest.Server.APM
{
    class GuiServerManager:ServerManager
    {
        
        private static GuiServerManager instance;
        public static GuiServerManager Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new GuiServerManager();
                }
                return instance;
            }
        }
        private GuiServerManager()
        {

        }
       public override void server_ClientConnected(object sender, TcpClientConnectedEventArgs e)
        {
            string str = string.Format("TCP client {0} has connected.", e.Session.RemoteEndPoint);
            Console.WriteLine(str);
            Logs.Info(str);
            OnMessageEvent(this,new MessageEventArgs (str));
            string pathAgv =ConstDefine.AGV_PATH;
            SendTo(e.Session.SessionKey,MessageType.AgvFile, pathAgv,false);
            //_server.SendTo()
        }

       public override void server_ClientDisconnected(object sender, TcpClientDisconnectedEventArgs e)
        {
            string str = string.Format("TCP client {0} has disconnected.", e.Session.RemoteEndPoint);
            Logs.Info(str);
            Console.WriteLine(str);
            OnMessageEvent(this, new MessageEventArgs(str));
        }

      public override void server_ClientDataReceived(object sender, TcpClientDataReceivedEventArgs e)
        {
            MessageType type = (MessageType)e.Data[e.DataOffset];
            var text = Encoding.UTF8.GetString(e.Data, e.DataOffset + 1, e.DataLength - 1);
            // OnDateMessageEvent(text);

            string str = string.Format("Client : {0}--> ", e.Session.RemoteEndPoint);
            OnMessageEvent(this, new MessageEventArgs(str));

            BaseMessage bm = BaseMessage.Factory(type, text);
            bm.ShowMessage += OnMessageEvent;
            bm.DataMessage += OnDataMessageEvent;
            bm.ReLoad += ReLoadDel;
            bm.Receive();
        }
        public void Broadcast(MessageType type, string json)
        {
           SendTo("", type, json, true);
            
        }

    }
}
