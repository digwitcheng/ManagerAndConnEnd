using AGV_V1._0.Algorithm;
using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using AGV_V1._0.Util;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class AgvFileMessage:FileMessage
    {
       
        public AgvFileMessage()
        {
            this.Type = MessageType.AgvFile;
        }
        public override BaseMessage Create(string msg)
        {
            AgvFileMessage dis = new AgvFileMessage();
            dis.Message = msg;
            return dis;
        }
    }
}
