using AGV_V1._0.Event;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class ArrivedMessage : TextMessage
    {       

        public override void Receive()
        {
            
        }
        public ArrivedMessage()
        {
            this.Type = MessageType.Arrived;
        }
        public override BaseMessage Create(string msg)
        {
            ArrivedMessage dis = new ArrivedMessage();
            dis.Message = msg;
            return dis;
        }
    }
}
