using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Event
{
    public enum MessageType:byte
    {
        DisConnect = 0, 
        Msg = 1, 
        MapFile = 2, 
        AgvFile = 3, 
        ReStart = 4, 
        Arrived = 5,
        None=6,
        Move=7
    }
}
