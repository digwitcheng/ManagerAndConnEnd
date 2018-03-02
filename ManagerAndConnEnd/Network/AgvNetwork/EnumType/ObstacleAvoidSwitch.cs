using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum ObstacleAvoidSwitch :byte
    {
        /// <summary>
        /// 避障关闭
        /// </summary>
        Off=0x01,
        /// <summary>
        /// 避障开启
        /// </summary>
        On=0x02
    }
}
