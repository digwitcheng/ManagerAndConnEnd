using AGV_V1._0.Event;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class MoveMessage:TextMessage
    {
       
        public override void Receive()
        {
            OnMessageEvent("Move");
            OnTransmitEvent(this, new MessageEventArgs(MessageType.Move, this.Message));
        }
        public MoveMessage()
        {
            this.Type = MessageType.Move;
        }
        public override BaseMessage Create(string msg)
        {
            MoveMessage dis = new MoveMessage();
            dis.Message = msg;
            return dis;
        }
    }
}
