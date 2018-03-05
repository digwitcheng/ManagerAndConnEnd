using AGV_V1._0.Event;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Queue
{
    class MyMessage
    {
        public MessageType Type { get; private set; }
        public string Message { get; set; }
        public MyMessage(MessageType type, string Message)
        {
            this.Type = type;
            this.Message = Message;
        }
    }
}
