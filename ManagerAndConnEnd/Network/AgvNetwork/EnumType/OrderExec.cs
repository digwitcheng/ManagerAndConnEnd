using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.EnumType
{
    enum OrderExecState:byte
    {
        Free = 0x00,       //空闲
        Init = 0x01,       //初始化
        Swerve = 0x02,     //转向
        LoadBag = 0x03,    //上包
        DropBag = 0x04,    //下包
        EmergStop = 0x05,  //急刹车
        Run = 0x06,        //运行
    }
}
