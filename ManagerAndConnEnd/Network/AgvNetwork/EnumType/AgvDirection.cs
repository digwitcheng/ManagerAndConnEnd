using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum MoveDirection:byte
    {
        /// <summary>
        /// 正向行驶
        /// </summary>
        Forward=1,
        /// <summary>
        /// 反向行驶（倒车行驶）
        /// </summary>
        Back=2
    }
}
