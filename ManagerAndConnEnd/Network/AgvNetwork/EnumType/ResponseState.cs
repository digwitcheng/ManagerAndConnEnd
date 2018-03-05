using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum ResponseState : byte
    {
        /// <summary>
        /// 正常
        /// </summary>
        Correct=0x00,
        /// <summary>
        /// 有误
        /// </summary>
        Error=0x01
    }
}
