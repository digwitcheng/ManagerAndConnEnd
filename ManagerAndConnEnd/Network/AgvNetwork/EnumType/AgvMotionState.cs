using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    /// <summary>
    /// 小车运动状态
    /// </summary>
    enum AgvMotionState:byte
    {
        /*
         * 没有初始化
                0x00 待初始化 
         * 停止状态
                0x01 停止（节点）
                0x02 停止（停在线路中）        
         * 运动行驶状态
                0x03 加速
                0x04 匀速
                0x05 减速        
         */

        /// <summary>
        /// 待初始化
        /// </summary>
        PendingInit =0x00,

        /// <summary>
        /// 停止（节点）
        /// </summary>
        StopedNode=0x01,

        /// <summary>
        /// 停止（停在线路中）
        /// </summary>
        StopedRoute=0x02,

        /// <summary>
        /// 加速
        /// </summary>
        SpeedUp=0x03,

        /// <summary>
        /// 匀速
        /// </summary>
        Uniform=0x04,

        /// <summary>
        /// 减速
        /// </summary>
        SpeedDown=0x05

    }
}
