using Astar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Event
{
    class SearchRouteEventArgs: EventArgs
    {
        public SendData sendData { get; set; }
        public SearchRouteEventArgs(SendData td)
        {
            this.sendData = td;
        }
    }
}
