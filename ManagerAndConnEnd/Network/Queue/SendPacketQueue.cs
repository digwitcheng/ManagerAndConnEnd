using AGV_V1._0;
using AGVSocket.Network.Packet;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Queue
{
    class SendPacketQueue:BaseQueue<SendBasePacket>
    {
        private static SendPacketQueue instance;
        public static SendPacketQueue Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new SendPacketQueue();
                }
                return instance;
            }
        }
        private SendPacketQueue() { }
    }
}
