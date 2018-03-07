using AGV_V1._0.Event;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class TextMessage:BaseMessage
    {
        public override BaseMessage Create(string msg)
        {
            return new TextMessage();
        }

        public override void Send(string sessionKey, TcpSocketServer _server)
        {

            List<byte> list = new List<byte>();
            list.Add((byte)this.Type);
            list.AddRange(Encoding.UTF8.GetBytes(this.Message));
            _server.Broadcast(list.ToArray());

        }

        public override void Receive()
        {
            OnMessageEvent("msg");
            OnTransmitEvent(this, new MessageEventArgs(MessageType.Msg, this.Message));
            // Console.WriteLine("MsgMessage:" + this.ShowMessage);
        }
    }
}
