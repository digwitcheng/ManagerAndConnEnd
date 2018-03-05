using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Event
{
    public class MessageEventArgs:EventArgs
    {
        public string Message { get; set; }
        public MessageType Type { get; set; }
        public MessageEventArgs(string msg)
        {
            Message = msg;
        }
        public MessageEventArgs(MessageType type, string msg)
        {
            Message = msg;
            Type = type;
        }
    }
}
