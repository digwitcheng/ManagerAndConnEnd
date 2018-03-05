using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class ErrorMessage:BaseMessage
    {
        public override void Send(string sessionKey, Cowboy.Sockets.TcpSocketServer _server)
        {
            
        }

        public override void Receive()
        {
            OnMessageEvent("错误的消息类型:"+this.Type+"消息:"+this.Message);
            Logs.Error("错误的消息类型:" + this.Type + "消息:" + this.Message);
        }
        public ErrorMessage()
        {
            this.Type = MessageType.None;
        }

        public override BaseMessage Create(string msg)
        {
            ErrorMessage dis = new ErrorMessage();
            dis.Message = msg;
            return dis;
        }
    }
}
