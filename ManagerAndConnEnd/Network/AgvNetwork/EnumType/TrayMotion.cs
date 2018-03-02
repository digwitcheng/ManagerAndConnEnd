using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum TrayMotion:byte
    {
        /// <summary>
        /// 无动作
        /// </summary>
        None=0x00,
        /// <summary>
        /// 左上包 (顶升式：顶升 )
        /// </summary>
        TopLeft  =0x01,   
        /// <summary>
        /// 皮带式（翻板式）：左下包 (顶升式：下降 )
        /// </summary>
        DownLeft = 0x02,
        /// <summary>
        /// 右上包
        /// </summary>
        TopRight = 0x03,
        /// <summary>
        /// 右下包
        /// </summary>
        DownRight = 0x04
    }
}
