using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum TrayState:byte
    {
        /*
         * 静止
                0x00 初始状态（无包裹）
                0x01 初始状态（光电被遮挡）
         * 运动
                0x02 上包中（或顶升中）
                0x03 上包完成
                0x04 下包中（或下降中）
                0x05 下包完成     
         */

        /// <summary>
        /// 初始状态（无包裹）
        /// </summary>
        InitNoBag =0x00,

        /// <summary>
        /// 初始状态（光电被遮挡）
        /// </summary>
        InitObscured= 0x01,

        /// <summary>
        /// 上包中（或顶升中）
        /// </summary>
        LoadingBag=0x02,

        /// <summary>
        /// 上包完成
        /// </summary>
        LoadedBag=0x03,

        /// <summary>
        /// 下包中（或下降中）
        /// </summary>
        DropBaging=0x04,

        /// <summary>
        /// 下包完成  
        /// </summary>
        DropBaged= 0x05

    }
}
