using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network
{
    public enum PacketType : byte 
    {

        //小车收到的报文类型
        Run = 0xA4,             //运行控制报文 0xA4
        Swerve = 0xA5,          //转向命令报文 0xA5
        Tray=0xA3,              //托盘控制报文 0xA3
        SysResponse=0xA9,       //上位机应答报文 0xA9

        //上位机收到的报文类型
        DoneReply=0xB1,         //操作完成回复报文 0xB1
        AgvInfo=0xB2,           //小车详情报文 0xB2
        AgvResponse=0xB9        //小车应答报文 0xB9

    }
}
