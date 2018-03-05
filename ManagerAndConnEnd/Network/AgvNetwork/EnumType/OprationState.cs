using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.MyException
{
    enum OprationState : byte
    {
        /*
            0x01 : 上包完成（顶升完成）
            0x02：下包完成（下降完成）
            0x03：转向完成
            0x04：充电完成
            0x05：紧急停车完成
         */

        /// <summary>
        /// 上包完成（顶升完成）
        /// </summary>
        Loaded=0x01,

        /// <summary>
        /// 下包完成（下降完成）
        /// </summary>
        Droped = 0X02,

        /// <summary>
        /// 转向完成
        /// </summary>
        Swerved = 0X03,

        /// <summary>
        /// 充电完成
        /// </summary>
        Charged = 0X04,

        /// <summary>
        /// 紧急停车完成
        /// </summary>
        EmergencyStop= 0x05
    }
}
